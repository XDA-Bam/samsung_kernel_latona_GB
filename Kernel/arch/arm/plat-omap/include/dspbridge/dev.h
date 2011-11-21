/*
 * dev.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Bridge Mini-driver device operations.
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

#ifndef DEV_
#define DEV_

/*  ----------------------------------- Module Dependent Headers */
#include <dspbridge/chnldefs.h>
#include <dspbridge/cmm.h>
#include <dspbridge/cod.h>
#include <dspbridge/dehdefs.h>
#include <dspbridge/nodedefs.h>
#include <dspbridge/dispdefs.h>
#include <dspbridge/wmd.h>
#include <dspbridge/dmm.h>
#include <dspbridge/host_os.h>

/*  ----------------------------------- This */
#include <dspbridge/devdefs.h>

/*
 *  ======== dev_brd_write_fxn ========
 *  Purpose:
 *      Exported function to be used as the COD write function.  This function
 *      is passed a handle to a DEV_hObject by ZL in pArb, then calls the
 *      device's bridge_brd_write() function.
 *  Parameters:
 *      pArb:           Handle to a Device Object.
 *      hDevContext:    Handle to mini-driver defined device info.
 *      dwDSPAddr:      Address on DSP board (Destination).
 *      pHostBuf:       Pointer to host buffer (Source).
 *      ul_num_bytes:     Number of bytes to transfer.
 *      ulMemType:      Memory space on DSP to which to transfer.
 *  Returns:
 *      Number of bytes written.  Returns 0 if the DEV_hObject passed in via
 *      pArb is invalid.
 *  Requires:
 *      DEV Initialized.
 *      pHostBuf != NULL
 *  Ensures:
 */
extern u32 dev_brd_write_fxn(void *pArb,
			     u32 ulDspAddr,
			     void *pHostBuf, u32 ul_num_bytes, u32 nMemSpace);

/*
 *  ======== dev_create_device ========
 *  Purpose:
 *      Called by the operating system to load the 'Bridge Mini Driver for a
 *      'Bridge device.
 *  Parameters:
 *      phDevObject:    Ptr to location to receive the device object handle.
 *      pstrWMDFileName: Name of WMD PE DLL file to load.  If the absolute
 *                      path is not provided, the file is loaded through
 *                      'Bridge's module search path.
 *      pHostConfig:    Host configuration information, to be passed down
 *                      to the WMD when bridge_dev_create() is called.
 *      pDspConfig:     DSP resources, to be passed down to the WMD when
 *                      bridge_dev_create() is called.
 *      dev_node_obj:       Platform (Windows) specific device node.
 *  Returns:
 *      0:            Module is loaded, device object has been created
 *      -ENOMEM:        Insufficient memory to create needed resources.
 *      DEV_E_NEWWMD:       The WMD was compiled for a newer version of WCD.
 *      DEV_E_NULLWMDINTF:  WMD passed back a NULL fxn Interface Struct Ptr
 *      DEV_E_NOCODMODULE:  No ZL file name was specified in the registry
 *                          for this dev_node_obj.
 *      LDR_E_FILEUNABLETOOPEN: Unable to open the specified WMD.
 *      LDR_E_NOMEMORY:         PELDR is out of resources.
 *      -EPERM:              Unable to find WMD entry point function.
 *      COD_E_NOZLFUNCTIONS:    One or more ZL functions exports not found.
 *      -ESPIPE:   Unable to load ZL DLL.
 *  Requires:
 *      DEV Initialized.
 *      phDevObject != NULL.
 *      pstrWMDFileName != NULL.
 *      pHostConfig != NULL.
 *      pDspConfig != NULL.
 *  Ensures:
 *      0:  *phDevObject will contain handle to the new device object.
 *      Otherwise, does not create the device object, ensures the WMD module is
 *      unloaded, and sets *phDevObject to NULL.
 */
extern int dev_create_device(OUT struct dev_object
				    **phDevObject,
				    IN CONST char *pstrWMDFileName,
				    struct cfg_devnode *dev_node_obj);

/*
 *  ======== dev_create_iva_device ========
 *  Purpose:
 *      Called by the operating system to load the 'Bridge Mini Driver for IVA.
 *  Parameters:
 *      phDevObject:    Ptr to location to receive the device object handle.
 *      pstrWMDFileName: Name of WMD PE DLL file to load.  If the absolute
 *                      path is not provided, the file is loaded through
 *                      'Bridge's module search path.
 *      pHostConfig:    Host configuration information, to be passed down
 *                      to the WMD when bridge_dev_create() is called.
 *      pDspConfig:     DSP resources, to be passed down to the WMD when
 *                      bridge_dev_create() is called.
 *      dev_node_obj:       Platform (Windows) specific device node.
 *  Returns:
 *      0:            Module is loaded, device object has been created
 *      -ENOMEM:        Insufficient memory to create needed resources.
 *      DEV_E_NEWWMD:       The WMD was compiled for a newer version of WCD.
 *      DEV_E_NULLWMDINTF:  WMD passed back a NULL fxn Interface Struct Ptr
 *      DEV_E_NOCODMODULE:  No ZL file name was specified in the registry
 *                          for this dev_node_obj.
 *      LDR_E_FILEUNABLETOOPEN: Unable to open the specified WMD.
 *      LDR_E_NOMEMORY:         PELDR is out of resources.
 *      -EPERM:              Unable to find WMD entry point function.
 *      COD_E_NOZLFUNCTIONS:    One or more ZL functions exports not found.
 *      -ESPIPE:   Unable to load ZL DLL.
 *  Requires:
 *      DEV Initialized.
 *      phDevObject != NULL.
 *      pstrWMDFileName != NULL.
 *      pHostConfig != NULL.
 *      pDspConfig != NULL.
 *  Ensures:
 *      0:  *phDevObject will contain handle to the new device object.
 *      Otherwise, does not create the device object, ensures the WMD module is
 *      unloaded, and sets *phDevObject to NULL.
 */
extern int dev_create_iva_device(OUT struct dev_object
					**phDevObject,
					IN CONST char *pstrWMDFileName,
					IN CONST struct cfg_hostres
					*pHostConfig,
					struct cfg_devnode *dev_node_obj);

/*
 *  ======== dev_create2 ========
 *  Purpose:
 *      After successful loading of the image from wcd_init_complete2
 *      (PROC Auto_Start) or proc_load this fxn is called. This creates
 *      the Node Manager and updates the DEV Object.
 *  Parameters:
 *      hdev_obj: Handle to device object created with dev_create_device().
 *  Returns:
 *      0:    Successful Creation of Node Manager
 *      -EPERM:  Some Error Occurred.
 *  Requires:
 *      DEV Initialized
 *      Valid hdev_obj
 *  Ensures:
 *      0 and hdev_obj->hnode_mgr != NULL
 *      else    hdev_obj->hnode_mgr == NULL
 */
extern int dev_create2(IN struct dev_object *hdev_obj);

/*
 *  ======== dev_destroy2 ========
 *  Purpose:
 *      Destroys the Node manager for this device.
 *  Parameters:
 *      hdev_obj: Handle to device object created with dev_create_device().
 *  Returns:
 *      0:    Successful Creation of Node Manager
 *      -EPERM:  Some Error Occurred.
 *  Requires:
 *      DEV Initialized
 *      Valid hdev_obj
 *  Ensures:
 *      0 and hdev_obj->hnode_mgr == NULL
 *      else    -EPERM.
 */
extern int dev_destroy2(IN struct dev_object *hdev_obj);

/*
 *  ======== dev_destroy_device ========
 *  Purpose:
 *      Destroys the channel manager for this device, if any, calls
 *      bridge_dev_destroy(), and then attempts to unload the WMD module.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *      -EPERM:      The WMD failed it's bridge_dev_destroy() function.
 *  Requires:
 *      DEV Initialized.
 *  Ensures:
 */
extern int dev_destroy_device(struct dev_object
				     *hdev_obj);

/*
 *  ======== dev_get_chnl_mgr ========
 *  Purpose:
 *      Retrieve the handle to the channel manager created for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *phMgr:         Ptr to location to store handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phMgr != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phMgr contains a handle to a channel manager object,
 *                      or NULL.
 *      else:           *phMgr is NULL.
 */
extern int dev_get_chnl_mgr(struct dev_object *hdev_obj,
				   OUT struct chnl_mgr **phMgr);

/*
 *  ======== dev_get_cmm_mgr ========
 *  Purpose:
 *      Retrieve the handle to the shared memory manager created for this
 *      device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *phMgr:         Ptr to location to store handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phMgr != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phMgr contains a handle to a channel manager object,
 *                      or NULL.
 *      else:           *phMgr is NULL.
 */
extern int dev_get_cmm_mgr(struct dev_object *hdev_obj,
				  OUT struct cmm_object **phMgr);

/*
 *  ======== dev_get_dmm_mgr ========
 *  Purpose:
 *      Retrieve the handle to the dynamic memory manager created for this
 *      device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *phMgr:         Ptr to location to store handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phMgr != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phMgr contains a handle to a channel manager object,
 *                      or NULL.
 *      else:           *phMgr is NULL.
 */
extern int dev_get_dmm_mgr(struct dev_object *hdev_obj,
				  OUT struct dmm_object **phMgr);

/*
 *  ======== dev_get_cod_mgr ========
 *  Purpose:
 *      Retrieve the COD manager create for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *phCodMgr:      Ptr to location to store handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phCodMgr != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phCodMgr contains a handle to a COD manager object.
 *      else:           *phCodMgr is NULL.
 */
extern int dev_get_cod_mgr(struct dev_object *hdev_obj,
				  OUT struct cod_manager **phCodMgr);

/*
 *  ======== dev_get_deh_mgr ========
 *  Purpose:
 *      Retrieve the DEH manager created for this device.
 *  Parameters:
 *      hdev_obj: Handle to device object created with dev_create_device().
 *      *phDehMgr:  Ptr to location to store handle.
 *  Returns:
 *      0:    Success.
 *      -EFAULT:   Invalid hdev_obj.
 *  Requires:
 *      phDehMgr != NULL.
 *      DEH Initialized.
 *  Ensures:
 *      0:    *phDehMgr contains a handle to a DEH manager object.
 *      else:       *phDehMgr is NULL.
 */
extern int dev_get_deh_mgr(struct dev_object *hdev_obj,
				  OUT struct deh_mgr **phDehMgr);

/*
 *  ======== dev_get_dev_node ========
 *  Purpose:
 *      Retrieve the platform specific device ID for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      phDevNode:      Ptr to location to get the device node handle.
 *  Returns:
 *      0:        In Win95, returns a DEVNODE in *dev_node_obj; In NT, ???
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phDevNode != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phDevNode contains a platform specific device ID;
 *      else:           *phDevNode is NULL.
 */
extern int dev_get_dev_node(struct dev_object *hdev_obj,
				   OUT struct cfg_devnode **phDevNode);

/*
 *  ======== dev_get_dev_type ========
 *  Purpose:
 *      Retrieve the platform specific device ID for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      phDevNode:      Ptr to location to get the device node handle.
 *  Returns:
 *      0:        Success
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phDevNode != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phDevNode contains a platform specific device ID;
 *      else:           *phDevNode is NULL.
 */
extern int dev_get_dev_type(struct dev_object *hdevObject,
					u8 *dev_type);

/*
 *  ======== dev_get_first ========
 *  Purpose:
 *      Retrieve the first Device Object handle from an internal linked list of
 *      of DEV_OBJECTs maintained by DEV.
 *  Parameters:
 *  Returns:
 *      NULL if there are no device objects stored; else
 *      a valid DEV_HOBJECT.
 *  Requires:
 *      No calls to dev_create_device or dev_destroy_device (which my modify the
 *      internal device object list) may occur between calls to dev_get_first
 *      and dev_get_next.
 *  Ensures:
 *      The DEV_HOBJECT returned is valid.
 *      A subsequent call to dev_get_next will return the next device object in
 *      the list.
 */
extern struct dev_object *dev_get_first(void);

/*
 *  ======== dev_get_intf_fxns ========
 *  Purpose:
 *      Retrieve the WMD interface function structure for the loaded WMD.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *ppIntfFxns:    Ptr to location to store fxn interface.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      ppIntfFxns != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *ppIntfFxns contains a pointer to the WMD interface;
 *      else:           *ppIntfFxns is NULL.
 */
extern int dev_get_intf_fxns(struct dev_object *hdev_obj,
			     OUT struct bridge_drv_interface **ppIntfFxns);

/*
 *  ======== dev_get_io_mgr ========
 *  Purpose:
 *      Retrieve the handle to the IO manager created for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      *phMgr:         Ptr to location to store handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phMgr != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phMgr contains a handle to an IO manager object.
 *      else:           *phMgr is NULL.
 */
extern int dev_get_io_mgr(struct dev_object *hdev_obj,
				 OUT struct io_mgr **phMgr);

/*
 *  ======== dev_get_next ========
 *  Purpose:
 *      Retrieve the next Device Object handle from an internal linked list of
 *      of DEV_OBJECTs maintained by DEV, after having previously called
 *      dev_get_first() and zero or more dev_get_next
 *  Parameters:
 *      hdev_obj: Handle to the device object returned from a previous
 *                  call to dev_get_first() or dev_get_next().
 *  Returns:
 *      NULL if there are no further device objects on the list or hdev_obj
 *      was invalid;
 *      else the next valid DEV_HOBJECT in the list.
 *  Requires:
 *      No calls to dev_create_device or dev_destroy_device (which my modify the
 *      internal device object list) may occur between calls to dev_get_first
 *      and dev_get_next.
 *  Ensures:
 *      The DEV_HOBJECT returned is valid.
 *      A subsequent call to dev_get_next will return the next device object in
 *      the list.
 */
extern struct dev_object *dev_get_next(struct dev_object
				       *hdev_obj);

/*
 *  ========= dev_get_msg_mgr ========
 *  Purpose:
 *      Retrieve the msg_ctrl Manager Handle from the DevObject.
 *  Parameters:
 *      hdev_obj: Handle to the Dev Object
 *      phMsgMgr:   Location where msg_ctrl Manager handle will be returned.
 *  Returns:
 *  Requires:
 *      DEV Initialized.
 *      Valid hdev_obj.
 *      phNodeMgr != NULL.
 *  Ensures:
 */
extern void dev_get_msg_mgr(struct dev_object *hdev_obj,
			    OUT struct msg_mgr **phMsgMgr);

/*
 *  ========= dev_get_node_manager ========
 *  Purpose:
 *      Retrieve the Node Manager Handle from the DevObject. It is an
 *      accessor function
 *  Parameters:
 *      hdev_obj:     Handle to the Dev Object
 *      phNodeMgr:      Location where Handle to the Node Manager will be
 *                      returned..
 *  Returns:
 *      0:        Success
 *      -EFAULT:    Invalid Dev Object handle.
 *  Requires:
 *      DEV Initialized.
 *      phNodeMgr is not null
 *  Ensures:
 *      0:        *phNodeMgr contains a handle to a Node manager object.
 *      else:           *phNodeMgr is NULL.
 */
extern int dev_get_node_manager(struct dev_object
				       *hdev_obj,
				       OUT struct node_mgr **phNodeMgr);

/*
 *  ======== dev_get_symbol ========
 *  Purpose:
 *      Get the value of a symbol in the currently loaded program.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      pstrSym:        Name of symbol to look up.
 *      pul_value:       Ptr to symbol value.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *      -ESPIPE:  Symbols have not been loaded onto the board.
 *      -ESPIPE:   The symbol could not be found.
 *  Requires:
 *      pstrSym != NULL.
 *      pul_value != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *pul_value contains the symbol value;
 */
extern int dev_get_symbol(struct dev_object *hdev_obj,
				 IN CONST char *pstrSym, OUT u32 * pul_value);

/*
 *  ======== dev_get_wmd_context ========
 *  Purpose:
 *      Retrieve the WMD Context handle, as returned by the WMD_Create fxn.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with dev_create_device()
 *      *phWmdContext:  Ptr to location to store context handle.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      phWmdContext != NULL.
 *      DEV Initialized.
 *  Ensures:
 *      0:        *phWmdContext contains context handle;
 *      else:           *phWmdContext is NULL;
 */
extern int dev_get_wmd_context(struct dev_object *hdev_obj,
				      OUT struct wmd_dev_context
				      **phWmdContext);

/*
 *  ======== dev_exit ========
 *  Purpose:
 *      Decrement reference count, and free resources when reference count is
 *      0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      DEV is initialized.
 *  Ensures:
 *      When reference count == 0, DEV's private resources are freed.
 */
extern void dev_exit(void);

/*
 *  ======== dev_init ========
 *  Purpose:
 *      Initialize DEV's private state, keeping a reference count on each call.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      TRUE: A requirement for the other public DEV functions.
 */
extern bool dev_init(void);

/*
 *  ======== dev_is_locked ========
 *  Purpose:
 *      Predicate function to determine if the device has been
 *      locked by a client for exclusive access.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *  Returns:
 *      0:        TRUE: device has been locked.
 *      0:     FALSE: device not locked.
 *      -EFAULT:    hdev_obj was invalid.
 *  Requires:
 *      DEV Initialized.
 *  Ensures:
 */
extern int dev_is_locked(IN struct dev_object *hdev_obj);

/*
 *  ======== dev_insert_proc_object ========
 *  Purpose:
 *      Inserts the Processor Object into the List of PROC Objects
 *      kept in the DEV Object
 *  Parameters:
 *      proc_obj:    Handle to the Proc Object
 *      hdev_obj      Handle to the Dev Object
 *      bAttachedNew    Specifies if there are already processors attached
 *  Returns:
 *      0:        Successfully inserted into the list
 *  Requires:
 *      proc_obj is not NULL
 *      hdev_obj is a valid handle to the DEV.
 *      DEV Initialized.
 *      List(of Proc object in Dev) Exists.
 *  Ensures:
 *      0 & the PROC Object is inserted and the list is not empty
 *  Details:
 *      If the List of Proc Object is empty bAttachedNew is TRUE, it indicated
 *      this is the first Processor attaching.
 *      If it is False, there are already processors attached.
 */
extern int dev_insert_proc_object(IN struct dev_object
					 *hdev_obj,
					 IN u32 proc_obj,
					 OUT bool *pbAlreadyAttached);

/*
 *  ======== dev_remove_proc_object ========
 *  Purpose:
 *      Search for and remove a Proc object from the given list maintained
 *      by the DEV
 *  Parameters:
 *      p_proc_object:        Ptr to ProcObject to insert.
 *      dev_obj:         Ptr to Dev Object where the list is.
 *      pbAlreadyAttached:  Ptr to return the bool
 *  Returns:
 *      0:            If successful.
 *      -EPERM           Failure to Remove the PROC Object from the list
 *  Requires:
 *      DevObject is Valid
 *      proc_obj != 0
 *      dev_obj->proc_list != NULL
 *      !LST_IS_EMPTY(dev_obj->proc_list)
 *      pbAlreadyAttached !=NULL
 *  Ensures:
 *  Details:
 *      List will be deleted when the DEV is destroyed.
 *
 */
extern int dev_remove_proc_object(struct dev_object
					 *hdev_obj, u32 proc_obj);

/*
 *  ======== dev_notify_clients ========
 *  Purpose:
 *      Notify all clients of this device of a change in device status.
 *      Clients may include multiple users of BRD, as well as CHNL.
 *      This function is asychronous, and may be called by a timer event
 *      set up by a watchdog timer.
 *  Parameters:
 *      hdev_obj:  Handle to device object created with dev_create_device().
 *      ulStatus:    A status word, most likely a BRD_STATUS.
 *  Returns:
 *      0:     All registered clients were asynchronously notified.
 *      -EINVAL:   Invalid hdev_obj.
 *  Requires:
 *      DEV Initialized.
 *  Ensures:
 *      0: Notifications are queued by the operating system to be
 *      delivered to clients.  This function does not ensure that
 *      the notifications will ever be delivered.
 */
extern int dev_notify_clients(struct dev_object *hdev_obj, u32 ulStatus);

/*
 *  ======== dev_remove_device ========
 *  Purpose:
 *      Destroys the Device Object created by dev_start_device.
 *  Parameters:
 *      dev_node_obj:       Device node as it is know to OS.
 *  Returns:
 *      0:        If success;
 *      <error code>    Otherwise.
 *  Requires:
 *  Ensures:
 */
extern int dev_remove_device(struct cfg_devnode *dev_node_obj);

/*
 *  ======== dev_set_chnl_mgr ========
 *  Purpose:
 *      Set the channel manager for this device.
 *  Parameters:
 *      hdev_obj:     Handle to device object created with
 *                      dev_create_device().
 *      hmgr:           Handle to a channel manager, or NULL.
 *  Returns:
 *      0:        Success.
 *      -EFAULT:    Invalid hdev_obj.
 *  Requires:
 *      DEV Initialized.
 *  Ensures:
 */
extern int dev_set_chnl_mgr(struct dev_object *hdev_obj,
				   struct chnl_mgr *hmgr);

/*
 *  ======== dev_set_msg_mgr ========
 *  Purpose:
 *      Set the Message manager for this device.
 *  Parameters:
 *      hdev_obj: Handle to device object created with dev_create_device().
 *      hmgr:       Handle to a message manager, or NULL.
 *  Returns:
 *  Requires:
 *      DEV Initialized.
 *  Ensures:
 */
extern void dev_set_msg_mgr(struct dev_object *hdev_obj, struct msg_mgr *hmgr);

/*
 *  ======== dev_start_device ========
 *  Purpose:
 *      Initializes the new device with the WinBRIDGE environment.  This
 *      involves querying CM for allocated resources, querying the registry
 *      for necessary dsp resources (requested in the INF file), and using
 *      this information to create a WinBRIDGE device object.
 *  Parameters:
 *      dev_node_obj:       Device node as it is know to OS.
 *  Returns:
 *      0:        If success;
 *      <error code>    Otherwise.
 *  Requires:
 *      DEV initialized.
 *  Ensures:
 */
extern int dev_start_device(struct cfg_devnode *dev_node_obj);

#endif /* DEV_ */
