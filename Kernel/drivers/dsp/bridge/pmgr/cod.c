/*
 * cod.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * This module implements DSP code management for the DSP/BIOS Bridge
 * environment. It is mostly a thin wrapper.
 *
 * This module provides an interface for loading both static and
 * dynamic code objects onto DSP systems.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/ldr.h>

/*  ----------------------------------- Platform Manager */
/* Include appropriate loader header file */
#include <dspbridge/dbll.h>

/*  ----------------------------------- This */
#include <dspbridge/cod.h>

/* magic number for handle validation */
#define MAGIC	 0xc001beef

/* macro to validate COD manager handles */
#define IS_VALID(h)    ((h) != NULL && (h)->ul_magic == MAGIC)

/*
 *  ======== cod_manager ========
 */
struct cod_manager {
	struct dbll_tar_obj *target;
	struct dbll_library_obj *base_lib;
	bool loaded;		/* Base library loaded? */
	u32 ul_entry;
	struct ldr_module *dll_obj;
	struct dbll_fxns fxns;
	struct dbll_attrs attrs;
	char sz_zl_file[COD_MAXPATHLENGTH];
	u32 ul_magic;
};

/*
 *  ======== cod_libraryobj ========
 */
struct cod_libraryobj {
	struct dbll_library_obj *dbll_lib;
	struct cod_manager *cod_mgr;
};

static u32 refs = 0L;

static struct dbll_fxns ldr_fxns = {
	(dbll_close_fxn) dbll_close,
	(dbll_create_fxn) dbll_create,
	(dbll_delete_fxn) dbll_delete,
	(dbll_exit_fxn) dbll_exit,
	(dbll_get_attrs_fxn) dbll_get_attrs,
	(dbll_get_addr_fxn) dbll_get_addr,
	(dbll_get_c_addr_fxn) dbll_get_c_addr,
	(dbll_get_sect_fxn) dbll_get_sect,
	(dbll_init_fxn) dbll_init,
	(dbll_load_fxn) dbll_load,
	(dbll_load_sect_fxn) dbll_load_sect,
	(dbll_open_fxn) dbll_open,
	(dbll_read_sect_fxn) dbll_read_sect,
	(dbll_set_attrs_fxn) dbll_set_attrs,
	(dbll_unload_fxn) dbll_unload,
	(dbll_unload_sect_fxn) dbll_unload_sect,
};

static bool no_op(void);

/*
 * File operations (originally were under kfile.c)
 */
static s32 cod_f_close(struct file *filp)
{
	/* Check for valid handle */
	if (!filp)
		return -EFAULT;

	filp_close(filp, NULL);

	/* we can't use 0 here */
	return 0;
}

static struct file *cod_f_open(CONST char *psz_file_name, CONST char *pszMode)
{
	mm_segment_t fs;
	struct file *filp;

	fs = get_fs();
	set_fs(get_ds());

	/* ignore given mode and open file as read-only */
	filp = filp_open(psz_file_name, O_RDONLY, 0);

	if (IS_ERR(filp))
		filp = NULL;

	set_fs(fs);

	return filp;
}

static s32 cod_f_read(void __user *pbuffer, s32 size, s32 cCount,
		      struct file *filp)
{
	/* check for valid file handle */
	if (!filp)
		return -EFAULT;

	if ((size > 0) && (cCount > 0) && pbuffer) {
		u32 dw_bytes_read;
		mm_segment_t fs;

		/* read from file */
		fs = get_fs();
		set_fs(get_ds());
		dw_bytes_read = filp->f_op->read(filp, pbuffer, size * cCount,
						 &(filp->f_pos));
		set_fs(fs);

		if (!dw_bytes_read)
			return -EBADF;

		return dw_bytes_read / size;
	}

	return -EINVAL;
}

static s32 cod_f_seek(struct file *filp, s32 lOffset, s32 cOrigin)
{
	loff_t dw_cur_pos;

	/* check for valid file handle */
	if (!filp)
		return -EFAULT;

	/* based on the origin flag, move the internal pointer */
	dw_cur_pos = filp->f_op->llseek(filp, lOffset, cOrigin);

	if ((s32) dw_cur_pos < 0)
		return -EPERM;

	/* we can't use 0 here */
	return 0;
}

static s32 cod_f_tell(struct file *filp)
{
	loff_t dw_cur_pos;

	if (!filp)
		return -EFAULT;

	/* Get current position */
	dw_cur_pos = filp->f_op->llseek(filp, 0, SEEK_CUR);

	if ((s32) dw_cur_pos < 0)
		return -EPERM;

	return dw_cur_pos;
}

/*
 *  ======== cod_close ========
 */
void cod_close(struct cod_libraryobj *lib)
{
	struct cod_manager *hmgr;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(lib != NULL);
	DBC_REQUIRE(IS_VALID(((struct cod_libraryobj *)lib)->cod_mgr));

	hmgr = lib->cod_mgr;
	hmgr->fxns.close_fxn(lib->dbll_lib);

	kfree(lib);
}

/*
 *  ======== cod_create ========
 *  Purpose:
 *      Create an object to manage code on a DSP system.
 *      This object can be used to load an initial program image with
 *      arguments that can later be expanded with
 *      dynamically loaded object files.
 *
 */
int cod_create(OUT struct cod_manager **phMgr, char *pstrDummyFile,
		      IN OPTIONAL CONST struct cod_attrs *attrs)
{
	struct cod_manager *mgr_new;
	struct dbll_attrs zl_attrs;
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(phMgr != NULL);

	/* assume failure */
	*phMgr = NULL;

	/* we don't support non-default attrs yet */
	if (attrs != NULL)
		return -ENOSYS;

	mgr_new = kzalloc(sizeof(struct cod_manager), GFP_KERNEL);
	if (mgr_new == NULL)
		return -ENOMEM;

	mgr_new->ul_magic = MAGIC;

	/* Set up loader functions */
	mgr_new->fxns = ldr_fxns;

	/* initialize the ZL module */
	mgr_new->fxns.init_fxn();

	zl_attrs.alloc = (dbll_alloc_fxn) no_op;
	zl_attrs.free = (dbll_free_fxn) no_op;
	zl_attrs.fread = (dbll_read_fxn) cod_f_read;
	zl_attrs.fseek = (dbll_seek_fxn) cod_f_seek;
	zl_attrs.ftell = (dbll_tell_fxn) cod_f_tell;
	zl_attrs.fclose = (dbll_f_close_fxn) cod_f_close;
	zl_attrs.fopen = (dbll_f_open_fxn) cod_f_open;
	zl_attrs.sym_lookup = NULL;
	zl_attrs.base_image = true;
	zl_attrs.log_write = NULL;
	zl_attrs.log_write_handle = NULL;
	zl_attrs.write = NULL;
	zl_attrs.rmm_handle = NULL;
	zl_attrs.input_params = NULL;
	zl_attrs.sym_handle = NULL;
	zl_attrs.sym_arg = NULL;

	mgr_new->attrs = zl_attrs;

	status = mgr_new->fxns.create_fxn(&mgr_new->target, &zl_attrs);

	if (DSP_FAILED(status)) {
		cod_delete(mgr_new);
		return -ESPIPE;
	}

	/* return the new manager */
	*phMgr = mgr_new;

	return 0;
}

/*
 *  ======== cod_delete ========
 *  Purpose:
 *      Delete a code manager object.
 */
void cod_delete(struct cod_manager *hmgr)
{
	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(hmgr));

	if (hmgr->base_lib) {
		if (hmgr->loaded)
			hmgr->fxns.unload_fxn(hmgr->base_lib, &hmgr->attrs);

		hmgr->fxns.close_fxn(hmgr->base_lib);
	}
	if (hmgr->target) {
		hmgr->fxns.delete_fxn(hmgr->target);
		hmgr->fxns.exit_fxn();
	}
	hmgr->ul_magic = ~MAGIC;
	kfree(hmgr);
}

/*
 *  ======== cod_exit ========
 *  Purpose:
 *      Discontinue usage of the COD module.
 *
 */
void cod_exit(void)
{
	DBC_REQUIRE(refs > 0);

	refs--;

	DBC_ENSURE(refs >= 0);
}

/*
 *  ======== cod_get_base_lib ========
 *  Purpose:
 *      Get handle to the base image DBL library.
 */
int cod_get_base_lib(struct cod_manager *cod_mgr_obj,
			    struct dbll_library_obj **plib)
{
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(cod_mgr_obj));
	DBC_REQUIRE(plib != NULL);

	*plib = (struct dbll_library_obj *)cod_mgr_obj->base_lib;

	return status;
}

/*
 *  ======== cod_get_base_name ========
 */
int cod_get_base_name(struct cod_manager *cod_mgr_obj, char *pszName,
			     u32 usize)
{
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(cod_mgr_obj));
	DBC_REQUIRE(pszName != NULL);

	if (usize <= COD_MAXPATHLENGTH)
		strncpy(pszName, cod_mgr_obj->sz_zl_file, usize);
	else
		status = -EPERM;

	return status;
}

/*
 *  ======== cod_get_entry ========
 *  Purpose:
 *      Retrieve the entry point of a loaded DSP program image
 *
 */
int cod_get_entry(struct cod_manager *cod_mgr_obj, u32 *pulEntry)
{
	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(cod_mgr_obj));
	DBC_REQUIRE(pulEntry != NULL);

	*pulEntry = cod_mgr_obj->ul_entry;

	return 0;
}

/*
 *  ======== cod_get_loader ========
 *  Purpose:
 *      Get handle to the DBLL loader.
 */
int cod_get_loader(struct cod_manager *cod_mgr_obj,
			  struct dbll_tar_obj **phLoader)
{
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(cod_mgr_obj));
	DBC_REQUIRE(phLoader != NULL);

	*phLoader = (struct dbll_tar_obj *)cod_mgr_obj->target;

	return status;
}

/*
 *  ======== cod_get_section ========
 *  Purpose:
 *      Retrieve the starting address and length of a section in the COFF file
 *      given the section name.
 */
int cod_get_section(struct cod_libraryobj *lib, IN char *pstrSect,
			   OUT u32 *puAddr, OUT u32 *puLen)
{
	struct cod_manager *cod_mgr_obj;
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(lib != NULL);
	DBC_REQUIRE(IS_VALID(lib->cod_mgr));
	DBC_REQUIRE(pstrSect != NULL);
	DBC_REQUIRE(puAddr != NULL);
	DBC_REQUIRE(puLen != NULL);

	*puAddr = 0;
	*puLen = 0;
	if (lib != NULL) {
		cod_mgr_obj = lib->cod_mgr;
		status = cod_mgr_obj->fxns.get_sect_fxn(lib->dbll_lib, pstrSect,
							puAddr, puLen);
	} else {
		status = -ESPIPE;
	}

	DBC_ENSURE(DSP_SUCCEEDED(status) || ((*puAddr == 0) && (*puLen == 0)));

	return status;
}

/*
 *  ======== cod_get_sym_value ========
 *  Purpose:
 *      Retrieve the value for the specified symbol. The symbol is first
 *      searched for literally and then, if not found, searched for as a
 *      C symbol.
 *
 */
int cod_get_sym_value(struct cod_manager *hmgr, char *pstrSym,
			     u32 *pul_value)
{
	struct dbll_sym_val *dbll_sym;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(hmgr));
	DBC_REQUIRE(pstrSym != NULL);
	DBC_REQUIRE(pul_value != NULL);

	dev_dbg(bridge, "%s: hmgr: %p pstrSym: %s pul_value: %p\n",
		__func__, hmgr, pstrSym, pul_value);
	if (hmgr->base_lib) {
		if (!hmgr->fxns.
		    get_addr_fxn(hmgr->base_lib, pstrSym, &dbll_sym)) {
			if (!hmgr->fxns.
			    get_c_addr_fxn(hmgr->base_lib, pstrSym, &dbll_sym))
				return -ESPIPE;
		}
	} else {
		return -ESPIPE;
	}

	*pul_value = dbll_sym->value;

	return 0;
}

/*
 *  ======== cod_init ========
 *  Purpose:
 *      Initialize the COD module's private state.
 *
 */
bool cod_init(void)
{
	bool ret = true;

	DBC_REQUIRE(refs >= 0);

	if (ret)
		refs++;

	DBC_ENSURE((ret && refs > 0) || (!ret && refs >= 0));
	return ret;
}

/*
 *  ======== cod_load_base ========
 *  Purpose:
 *      Load the initial program image, optionally with command-line arguments,
 *      on the DSP system managed by the supplied handle. The program to be
 *      loaded must be the first element of the args array and must be a fully
 *      qualified pathname.
 *  Details:
 *      if nArgc doesn't match the number of arguments in the aArgs array, the
 *      aArgs array is searched for a NULL terminating entry, and argc is
 *      recalculated to reflect this.  In this way, we can support NULL
 *      terminating aArgs arrays, if nArgc is very large.
 */
int cod_load_base(struct cod_manager *hmgr, u32 nArgc, char *aArgs[],
			 cod_writefxn pfn_write, void *pArb, char *envp[])
{
	dbll_flags flags;
	struct dbll_attrs save_attrs;
	struct dbll_attrs new_attrs;
	int status;
	u32 i;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(hmgr));
	DBC_REQUIRE(nArgc > 0);
	DBC_REQUIRE(aArgs != NULL);
	DBC_REQUIRE(aArgs[0] != NULL);
	DBC_REQUIRE(pfn_write != NULL);
	DBC_REQUIRE(hmgr->base_lib != NULL);

	/*
	 *  Make sure every argv[] stated in argc has a value, or change argc to
	 *  reflect true number in NULL terminated argv array.
	 */
	for (i = 0; i < nArgc; i++) {
		if (aArgs[i] == NULL) {
			nArgc = i;
			break;
		}
	}

	/* set the write function for this operation */
	hmgr->fxns.get_attrs_fxn(hmgr->target, &save_attrs);

	new_attrs = save_attrs;
	new_attrs.write = (dbll_write_fxn) pfn_write;
	new_attrs.input_params = pArb;
	new_attrs.alloc = (dbll_alloc_fxn) no_op;
	new_attrs.free = (dbll_free_fxn) no_op;
	new_attrs.log_write = NULL;
	new_attrs.log_write_handle = NULL;

	/* Load the image */
	flags = DBLL_CODE | DBLL_DATA | DBLL_SYMB;
	status = hmgr->fxns.load_fxn(hmgr->base_lib, flags, &new_attrs,
				     &hmgr->ul_entry);
	if (DSP_FAILED(status))
		hmgr->fxns.close_fxn(hmgr->base_lib);

	if (DSP_SUCCEEDED(status))
		hmgr->loaded = true;
	else
		hmgr->base_lib = NULL;

	return status;
}

/*
 *  ======== cod_open ========
 *      Open library for reading sections.
 */
int cod_open(struct cod_manager *hmgr, IN char *pszCoffPath,
		    cod_flags flags, struct cod_libraryobj **pLib)
{
	int status = 0;
	struct cod_libraryobj *lib = NULL;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(hmgr));
	DBC_REQUIRE(pszCoffPath != NULL);
	DBC_REQUIRE(flags == COD_NOLOAD || flags == COD_SYMB);
	DBC_REQUIRE(pLib != NULL);

	*pLib = NULL;

	lib = kzalloc(sizeof(struct cod_libraryobj), GFP_KERNEL);
	if (lib == NULL)
		status = -ENOMEM;

	if (DSP_SUCCEEDED(status)) {
		lib->cod_mgr = hmgr;
		status = hmgr->fxns.open_fxn(hmgr->target, pszCoffPath, flags,
					     &lib->dbll_lib);
		if (DSP_SUCCEEDED(status))
			*pLib = lib;
	}

	if (DSP_FAILED(status))
		pr_err("%s: error status %i, pszCoffPath: %s flags: 0x%x\n",
		       __func__, status, pszCoffPath, flags);
	return status;
}

/*
 *  ======== cod_open_base ========
 *  Purpose:
 *      Open base image for reading sections.
 */
int cod_open_base(struct cod_manager *hmgr, IN char *pszCoffPath,
			 dbll_flags flags)
{
	int status = 0;
	struct dbll_library_obj *lib;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(IS_VALID(hmgr));
	DBC_REQUIRE(pszCoffPath != NULL);

	/* if we previously opened a base image, close it now */
	if (hmgr->base_lib) {
		if (hmgr->loaded) {
			hmgr->fxns.unload_fxn(hmgr->base_lib, &hmgr->attrs);
			hmgr->loaded = false;
		}
		hmgr->fxns.close_fxn(hmgr->base_lib);
		hmgr->base_lib = NULL;
	}
	status = hmgr->fxns.open_fxn(hmgr->target, pszCoffPath, flags, &lib);
	if (DSP_SUCCEEDED(status)) {
		/* hang onto the library for subsequent sym table usage */
		hmgr->base_lib = lib;
		strncpy(hmgr->sz_zl_file, pszCoffPath, COD_MAXPATHLENGTH - 1);
		hmgr->sz_zl_file[COD_MAXPATHLENGTH - 1] = '\0';
	}

	if (DSP_FAILED(status))
		pr_err("%s: error status %i pszCoffPath: %s\n", __func__,
		       status, pszCoffPath);
	return status;
}

/*
 *  ======== cod_read_section ========
 *  Purpose:
 *      Retrieve the content of a code section given the section name.
 */
int cod_read_section(struct cod_libraryobj *lib, IN char *pstrSect,
			    OUT char *pstrContent, IN u32 cContentSize)
{
	int status = 0;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(lib != NULL);
	DBC_REQUIRE(IS_VALID(lib->cod_mgr));
	DBC_REQUIRE(pstrSect != NULL);
	DBC_REQUIRE(pstrContent != NULL);

	if (lib != NULL)
		status =
		    lib->cod_mgr->fxns.read_sect_fxn(lib->dbll_lib, pstrSect,
						     pstrContent, cContentSize);
	else
		status = -ESPIPE;

	return status;
}

/*
 *  ======== no_op ========
 *  Purpose:
 *      No Operation.
 *
 */
static bool no_op(void)
{
	return true;
}
