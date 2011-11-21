/*
 * dbdcd.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Defines the DSP/BIOS Bridge Configuration Database (DCD) API.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DBDCD_
#define DBDCD_

#include <dspbridge/dbdcddef.h>
#include <dspbridge/host_os.h>
#include <dspbridge/nldrdefs.h>

/*
 *  ======== dcd_auto_register ========
 *  Purpose:
 *      This function automatically registers DCD objects specified in a
 *      special COFF section called ".dcd_register"
 *  Parameters:
 *      hdcd_mgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing DCD
 *                              objects to be registered.
 *  Returns:
 *      0:                Success.
 *      -EACCES: Unable to find auto-registration section.
 *      -EACCES:       Unable to read object code section.
 *      -EACCES:       Unable to load code base.
 *      -EFAULT:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto registration.
 */
extern int dcd_auto_register(IN struct dcd_manager *hdcd_mgr,
				    IN char *pszCoffPath);

/*
 *  ======== dcd_auto_unregister ========
 *  Purpose:
 *      This function automatically unregisters DCD objects specified in a
 *      special COFF section called ".dcd_register"
 *  Parameters:
 *      hdcd_mgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing
 *                              DCD objects to be unregistered.
 *  Returns:
 *      0:                Success.
 *      -EACCES: Unable to find auto-registration section.
 *      -EACCES:       Unable to read object code section.
 *      -EACCES:       Unable to load code base.
 *      -EFAULT:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto unregistration.
 */
extern int dcd_auto_unregister(IN struct dcd_manager *hdcd_mgr,
				      IN char *pszCoffPath);

/*
 *  ======== dcd_create_manager ========
 *  Purpose:
 *      This function creates a DCD module manager.
 *  Parameters:
 *      pszZlDllName:   Pointer to a DLL name string.
 *      phDcdMgr:       A pointer to a DCD manager handle.
 *  Returns:
 *      0:        Success.
 *      -ENOMEM:    Unable to allocate memory for DCD manager handle.
 *      -EPERM:      General failure.
 *  Requires:
 *      DCD initialized.
 *      pszZlDllName is non-NULL.
 *      phDcdMgr is non-NULL.
 *  Ensures:
 *      A DCD manager handle is created.
 */
extern int dcd_create_manager(IN char *pszZlDllName,
				     OUT struct dcd_manager **phDcdMgr);

/*
 *  ======== dcd_destroy_manager ========
 *  Purpose:
 *      This function destroys a DCD module manager.
 *  Parameters:
 *      hdcd_mgr:        A DCD manager handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid DCD manager handle.
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 */
extern int dcd_destroy_manager(IN struct dcd_manager *hdcd_mgr);

/*
 *  ======== dcd_enumerate_object ========
 *  Purpose:
 *      This function enumerates currently visible DSP/BIOS Bridge objects
 *      and returns the UUID and type of each enumerated object.
 *  Parameters:
 *      cIndex:             The object enumeration index.
 *      obj_type:            Type of object to enumerate.
 *      uuid_obj:              Pointer to a dsp_uuid object.
 *  Returns:
 *      0:            Success.
 *      -EPERM:          Unable to enumerate through the DCD database.
 *      ENODATA:  Enumeration completed. This is not an error code.
 *  Requires:
 *      DCD initialized.
 *      uuid_obj is a valid pointer.
 *  Ensures:
 *  Details:
 *      This function can be used in conjunction with dcd_get_object_def to
 *      retrieve object properties.
 */
extern int dcd_enumerate_object(IN s32 cIndex,
				       IN enum dsp_dcdobjtype obj_type,
				       OUT struct dsp_uuid *uuid_obj);

/*
 *  ======== dcd_exit ========
 *  Purpose:
 *      This function cleans up the DCD module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 */
extern void dcd_exit(void);

/*
 *  ======== dcd_get_dep_libs ========
 *  Purpose:
 *      Given the uuid of a library and size of array of uuids, this function
 *      fills the array with the uuids of all dependent libraries of the input
 *      library.
 *  Parameters:
 *      hdcd_mgr: A DCD manager handle.
 *      uuid_obj: Pointer to a dsp_uuid for a library.
 *      numLibs: Size of uuid array (number of library uuids).
 *      pDepLibUuids: Array of dependent library uuids to be filled in.
 *      pPersistentDepLibs: Array indicating if corresponding lib is persistent.
 *      phase: phase to obtain correct input library
 *  Returns:
 *      0: Success.
 *      -ENOMEM: Memory allocation failure.
 *      -EACCES: Failure to read section containing library info.
 *      -EPERM: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hdcd_mgr.
 *      uuid_obj != NULL
 *      pDepLibUuids != NULL.
 *  Ensures:
 */
extern int dcd_get_dep_libs(IN struct dcd_manager *hdcd_mgr,
				   IN struct dsp_uuid *uuid_obj,
				   u16 numLibs,
				   OUT struct dsp_uuid *pDepLibUuids,
				   OUT bool *pPersistentDepLibs,
				   IN enum nldr_phase phase);

/*
 *  ======== dcd_get_num_dep_libs ========
 *  Purpose:
 *      Given the uuid of a library, determine its number of dependent
 *      libraries.
 *  Parameters:
 *      hdcd_mgr:        A DCD manager handle.
 *      uuid_obj:          Pointer to a dsp_uuid for a library.
 *      pNumLibs:       Size of uuid array (number of library uuids).
 *      pNumPersLibs:   number of persistent dependent library.
 *      phase:          Phase to obtain correct input library
 *  Returns:
 *      0: Success.
 *      -ENOMEM: Memory allocation failure.
 *      -EACCES: Failure to read section containing library info.
 *      -EPERM: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hdcd_mgr.
 *      uuid_obj != NULL
 *      pNumLibs != NULL.
 *  Ensures:
 */
extern int dcd_get_num_dep_libs(IN struct dcd_manager *hdcd_mgr,
				       IN struct dsp_uuid *uuid_obj,
				       OUT u16 *pNumLibs,
				       OUT u16 *pNumPersLibs,
				       IN enum nldr_phase phase);

/*
 *  ======== dcd_get_library_name ========
 *  Purpose:
 *      This function returns the name of a (dynamic) library for a given
 *      UUID.
 *  Parameters:
 *      hdcd_mgr: A DCD manager handle.
 *      uuid_obj:	Pointer to a dsp_uuid that represents a unique DSP/BIOS
 *                      Bridge object.
 *      pstrLibName: Buffer to hold library name.
 *      pdwSize: Contains buffer size. Set to string size on output.
 *      phase:          Which phase to load
 *      phase_split:    Are phases in multiple libraries
 *  Returns:
 *      0: Success.
 *      -EPERM: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hdcd_mgr.
 *      pstrLibName != NULL.
 *      uuid_obj != NULL
 *      pdwSize != NULL.
 *  Ensures:
 */
extern int dcd_get_library_name(IN struct dcd_manager *hdcd_mgr,
				       IN struct dsp_uuid *uuid_obj,
				       IN OUT char *pstrLibName,
				       IN OUT u32 *pdwSize,
				       IN enum nldr_phase phase,
				       OUT bool *phase_split);

/*
 *  ======== dcd_get_object_def ========
 *  Purpose:
 *      This function returns the properties/attributes of a DSP/BIOS Bridge
 *      object.
 *  Parameters:
 *      hdcd_mgr:            A DCD manager handle.
 *      uuid_obj:              Pointer to a dsp_uuid that represents a unique
 *                          DSP/BIOS Bridge object.
 *      obj_type:            The type of DSP/BIOS Bridge object to be
 *                          referenced (node, processor, etc).
 *      pObjDef:            Pointer to an object definition structure. A
 *                          union of various possible DCD object types.
 *  Returns:
 *      0: Success.
 *      -EACCES:  Unable to parse content of object code section.
 *      -EACCES:   Unable to read object code section.
 *      -EACCES:    Unable to access object code section.
 *      -EACCES:   Unable to load code base.
 *      -EPERM:          General failure.
 *      -EFAULT:        Invalid DCD_HMANAGER handle.
 *  Requires:
 *      DCD initialized.
 *      pObjUuid is non-NULL.
 *      pObjDef is non-NULL.
 *  Ensures:
 */
extern int dcd_get_object_def(IN struct dcd_manager *hdcd_mgr,
				     IN struct dsp_uuid *pObjUuid,
				     IN enum dsp_dcdobjtype obj_type,
				     OUT struct dcd_genericobj *pObjDef);

/*
 *  ======== dcd_get_objects ========
 *  Purpose:
 *      This function finds all DCD objects specified in a special
 *      COFF section called ".dcd_register", and for each object,
 *      call a "register" function.  The "register" function may perform
 *      various actions, such as 1) register nodes in the node database, 2)
 *      unregister nodes from the node database, and 3) add overlay nodes.
 *  Parameters:
 *      hdcd_mgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing DCD
 *                              objects.
 *      registerFxn:            Callback fxn to be applied on each located
 *                              DCD object.
 *      handle:                 Handle to pass to callback.
 *  Returns:
 *      0:                Success.
 *      -EACCES: Unable to find .dcd_register section.
 *      -EACCES:       Unable to read object code section.
 *      -EACCES:       Unable to load code base.
 *      -EFAULT:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto registration.
 */
extern int dcd_get_objects(IN struct dcd_manager *hdcd_mgr,
				  IN char *pszCoffPath,
				  dcd_registerfxn registerFxn, void *handle);

/*
 *  ======== dcd_init ========
 *  Purpose:
 *      This function initializes DCD.
 *  Parameters:
 *  Returns:
 *      FALSE:  Initialization failed.
 *      TRUE:   Initialization succeeded.
 *  Requires:
 *  Ensures:
 *      DCD initialized.
 */
extern bool dcd_init(void);

/*
 *  ======== dcd_register_object ========
 *  Purpose:
 *      This function registers a DSP/BIOS Bridge object in the DCD database.
 *  Parameters:
 *      uuid_obj:          Pointer to a dsp_uuid that identifies a DSP/BIOS
 *                      Bridge object.
 *      obj_type:        Type of object.
 *      psz_path_name:    Path to the object's COFF file.
 *  Returns:
 *      0:        Success.
 *      -EPERM:      Failed to register object.
 *  Requires:
 *      DCD initialized.
 *      uuid_obj and szPathName are non-NULL values.
 *      obj_type is a valid type value.
 *  Ensures:
 */
extern int dcd_register_object(IN struct dsp_uuid *uuid_obj,
				      IN enum dsp_dcdobjtype obj_type,
				      IN char *psz_path_name);

/*
 *  ======== dcd_unregister_object ========
 *  Purpose:
 *      This function de-registers a valid DSP/BIOS Bridge object from the DCD
 *      database.
 *  Parameters:
 *      uuid_obj:      Pointer to a dsp_uuid that identifies a DSP/BIOS Bridge
 *                  object.
 *      obj_type:    Type of object.
 *  Returns:
 *      0:    Success.
 *      -EPERM:  Unable to de-register the specified object.
 *  Requires:
 *      DCD initialized.
 *      uuid_obj is a non-NULL value.
 *      obj_type is a valid type value.
 *  Ensures:
 */
extern int dcd_unregister_object(IN struct dsp_uuid *uuid_obj,
					IN enum dsp_dcdobjtype obj_type);

#endif /* _DBDCD_H */
