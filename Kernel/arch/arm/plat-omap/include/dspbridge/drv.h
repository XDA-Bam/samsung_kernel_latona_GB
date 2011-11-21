/*
 * drv.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DRV Resource allocation module. Driver Object gets Created
 * at the time of Loading. It holds the List of Device Objects
 * in the system.
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

#ifndef DRV_
#define DRV_

#include <dspbridge/devdefs.h>

#include <dspbridge/drvdefs.h>
#include <linux/idr.h>

#define DRV_ASSIGN     1
#define DRV_RELEASE    0

/* Provide the DSP Internal memory windows that can be accessed from L3 address
 * space */

#define OMAP_GEM_BASE   0x107F8000
#define OMAP_DSP_SIZE   0x00720000

/* MEM1 is L2 RAM + L2 Cache space */
#define OMAP_DSP_MEM1_BASE 0x5C7F8000
#define OMAP_DSP_MEM1_SIZE 0x18000
#define OMAP_DSP_GEM1_BASE 0x107F8000

/* MEM2 is L1P RAM/CACHE space */
#define OMAP_DSP_MEM2_BASE 0x5CE00000
#define OMAP_DSP_MEM2_SIZE 0x8000
#define OMAP_DSP_GEM2_BASE 0x10E00000

/* MEM3 is L1D RAM/CACHE space */
#define OMAP_DSP_MEM3_BASE 0x5CF04000
#define OMAP_DSP_MEM3_SIZE 0x14000
#define OMAP_DSP_GEM3_BASE 0x10F04000

#define OMAP_IVA2_PRM_BASE 0x48306000
#define OMAP_IVA2_PRM_SIZE 0x1000

#define OMAP_IVA2_CM_BASE 0x48004000
#define OMAP_IVA2_CM_SIZE 0x1000

#define OMAP_PER_CM_BASE 0x48005000
#define OMAP_PER_CM_SIZE 0x1000

#define OMAP_PER_PRM_BASE 0x48307000
#define OMAP_PER_PRM_SIZE 0x1000

#define OMAP_CORE_PRM_BASE 0x48306A00
#define OMAP_CORE_PRM_SIZE 0x1000

#define OMAP_SYSC_BASE 0x48002000
#define OMAP_SYSC_SIZE 0x1000

#define OMAP_DMMU_BASE 0x5D000000
#define OMAP_DMMU_SIZE 0x1000

#define OMAP_WDT3_BASE 0x49030000
#define OMAP_WDT3_SIZE 0x1000

#define OMAP_PRCM_VDD1_DOMAIN 1
#define OMAP_PRCM_VDD2_DOMAIN 2

/* GPP PROCESS CLEANUP Data structures */

/* New structure (member of process context) abstracts NODE resource info */
struct node_res_object {
	void *hnode;
	s32 node_allocated;	/* Node status */
	s32 heap_allocated;	/* Heap status */
	s32 streams_allocated;	/* Streams status */
	int id;
};

/* Used for DMM mapped memory accounting */
struct dmm_map_object {
	struct list_head link;
	u32 dsp_addr;
};

/* Used for DMM reserved memory accounting */
struct dmm_rsv_object {
	struct list_head link;
	u32 dsp_reserved_addr;
};

/* New structure (member of process context) abstracts DMM resource info */
struct dspheap_res_object {
	s32 heap_allocated;	/* DMM status */
	u32 ul_mpu_addr;
	u32 ul_dsp_addr;
	u32 ul_dsp_res_addr;
	u32 heap_size;
	bhandle hprocessor;
	struct dspheap_res_object *next;
};

/* New structure (member of process context) abstracts stream resource info */
struct strm_res_object {
	s32 stream_allocated;	/* Stream status */
	void *hstream;
	u32 num_bufs;
	u32 dir;
	int id;
};

/* Overall Bridge process resource usage state */
enum gpp_proc_res_state {
	PROC_RES_ALLOCATED,
	PROC_RES_FREED
};

/* Bridge Data */
struct drv_data {
	char *base_img;
	s32 shm_size;
	int tc_wordswapon;
	void *drv_object;
	void *dev_object;
	void *mgr_object;
};

/* Process Context */
struct process_context {
	/* Process State */
	enum gpp_proc_res_state res_state;

	/* Handle to Processor */
	void *hprocessor;

	/* DSP Node resources */
	struct idr *node_idp;

	/* DMM mapped memory resources */
	struct list_head dmm_map_list;
	spinlock_t dmm_map_lock;

	/* DMM reserved memory resources */
	struct list_head dmm_rsv_list;
	spinlock_t dmm_rsv_lock;

	/* DSP Heap resources */
	struct dspheap_res_object *pdspheap_list;

	/* Stream resources */
	struct idr *strm_idp;
};

/*
 *  ======== drv_create ========
 *  Purpose:
 *      Creates the Driver Object. This is done during the driver loading.
 *      There is only one Driver Object in the DSP/BIOS Bridge.
 *  Parameters:
 *      phDrvObject:    Location to store created DRV Object handle.
 *  Returns:
 *      0:        Sucess
 *      -ENOMEM:    Failed in Memory allocation
 *      -EPERM:      General Failure
 *  Requires:
 *      DRV Initialized (refs > 0 )
 *      phDrvObject != NULL.
 *  Ensures:
 *      0:        - *phDrvObject is a valid DRV interface to the device.
 *                      - List of DevObject Created and Initialized.
 *                      - List of dev_node String created and intialized.
 *                      - Registry is updated with the DRV Object.
 *      !0:       DRV Object not created
 *  Details:
 *      There is one Driver Object for the Driver representing
 *      the driver itself. It contains the list of device
 *      Objects and the list of Device Extensions in the system.
 *      Also it can hold other neccessary
 *      information in its storage area.
 */
extern int drv_create(struct drv_object **phDrvObject);

/*
 *  ======== drv_destroy ========
 *  Purpose:
 *      destroys the Dev Object list, DrvExt list
 *      and destroy the DRV object
 *      Called upon driver unLoading.or unsuccesful loading of the driver.
 *  Parameters:
 *      hdrv_obj:     Handle to Driver object .
 *  Returns:
 *      0:        Success.
 *      -EPERM:      Failed to destroy DRV Object
 *  Requires:
 *      DRV Initialized (cRegs > 0 )
 *      hdrv_obj is not NULL and a valid DRV handle .
 *      List of DevObject is Empty.
 *      List of DrvExt is Empty
 *  Ensures:
 *      0:        - DRV Object destroyed and hdrv_obj is not a valid
 *                        DRV handle.
 *                      - Registry is updated with "0" as the DRV Object.
 */
extern int drv_destroy(struct drv_object *hdrv_obj);

/*
 *  ======== drv_exit ========
 *  Purpose:
 *      Exit the DRV module, freeing any modules initialized in drv_init.
 *  Parameters:
 *  Returns:
 *  Requires:
 *  Ensures:
 */
extern void drv_exit(void);

/*
 *  ======== drv_get_first_dev_object ========
 *  Purpose:
 *      Returns the Ptr to the FirstDev Object in the List
 *  Parameters:
 *  Requires:
 *      DRV Initialized
 *  Returns:
 *      dw_dev_object:  Ptr to the First Dev Object as a u32
 *      0 if it fails to retrieve the First Dev Object
 *  Ensures:
 */
extern u32 drv_get_first_dev_object(void);

/*
 *  ======== drv_get_first_dev_extension ========
 *  Purpose:
 *      Returns the Ptr to the First Device Extension in the List
 *  Parameters:
 *  Requires:
 *      DRV Initialized
 *  Returns:
 *      dw_dev_extension:     Ptr to the First Device Extension as a u32
 *      0:                  Failed to Get the Device Extension
 *  Ensures:
 */
extern u32 drv_get_first_dev_extension(void);

/*
 *  ======== drv_get_dev_object ========
 *  Purpose:
 *      Given a index, returns a handle to DevObject from the list
 *  Parameters:
 *      hdrv_obj:     Handle to the Manager
 *      phDevObject:    Location to store the Dev Handle
 *  Requires:
 *      DRV Initialized
 *      index >= 0
 *      hdrv_obj is not NULL and Valid DRV Object
 *      phDevObject is not NULL
 *      Device Object List not Empty
 *  Returns:
 *      0:        Success
 *      -EPERM:      Failed to Get the Dev Object
 *  Ensures:
 *      0:        *phDevObject != NULL
 *      -EPERM:      *phDevObject = NULL
 */
extern int drv_get_dev_object(u32 index,
				     struct drv_object *hdrv_obj,
				     struct dev_object **phDevObject);

/*
 *  ======== drv_get_next_dev_object ========
 *  Purpose:
 *      Returns the Ptr to the Next Device Object from the the List
 *  Parameters:
 *      hdev_obj:     Handle to the Device Object
 *  Requires:
 *      DRV Initialized
 *      hdev_obj != 0
 *  Returns:
 *      dw_dev_object:    Ptr to the Next Dev Object as a u32
 *      0:              If it fail to get the next Dev Object.
 *  Ensures:
 */
extern u32 drv_get_next_dev_object(u32 hdev_obj);

/*
 *  ======== drv_get_next_dev_extension ========
 *  Purpose:
 *      Returns the Ptr to the Next Device Extension from the the List
 *  Parameters:
 *      hDevExtension:      Handle to the Device Extension
 *  Requires:
 *      DRV Initialized
 *      hDevExtension != 0.
 *  Returns:
 *      dw_dev_extension:     Ptr to the Next Dev Extension
 *      0:                  If it fail to Get the next Dev Extension
 *  Ensures:
 */
extern u32 drv_get_next_dev_extension(u32 hDevExtension);

/*
 *  ======== drv_init ========
 *  Purpose:
 *      Initialize the DRV module.
 *  Parameters:
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 */
extern int drv_init(void);

/*
 *  ======== drv_insert_dev_object ========
 *  Purpose:
 *      Insert a DeviceObject into the list of Driver object.
 *  Parameters:
 *      hdrv_obj:     Handle to DrvObject
 *      hdev_obj:     Handle to DeviceObject to insert.
 *  Returns:
 *      0:        If successful.
 *      -EPERM:      General Failure:
 *  Requires:
 *      hdrv_obj != NULL and Valid DRV Handle.
 *      hdev_obj != NULL.
 *  Ensures:
 *      0:        Device Object is inserted and the List is not empty.
 */
extern int drv_insert_dev_object(struct drv_object *hdrv_obj,
					struct dev_object *hdev_obj);

/*
 *  ======== drv_remove_dev_object ========
 *  Purpose:
 *      Search for and remove a Device object from the given list of Device Obj
 *      objects.
 *  Parameters:
 *      hdrv_obj:     Handle to DrvObject
 *      hdev_obj:     Handle to DevObject to Remove
 *  Returns:
 *      0:        Success.
 *      -EPERM:      Unable to find dev_obj.
 *  Requires:
 *      hdrv_obj != NULL and a Valid DRV Handle.
 *      hdev_obj != NULL.
 *      List exists and is not empty.
 *  Ensures:
 *      List either does not exist (NULL), or is not empty if it does exist.
 */
extern int drv_remove_dev_object(struct drv_object *hdrv_obj,
					struct dev_object *hdev_obj);

/*
 *  ======== drv_request_resources ========
 *  Purpose:
 *      Assigns the Resources or Releases them.
 *  Parameters:
 *      dw_context:          Path to the driver Registry Key.
 *      pDevNodeString:     Ptr to dev_node String stored in the Device Ext.
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 *      The Resources are assigned based on Bus type.
 *      The hardware is initialized. Resource information is
 *      gathered from the Registry(ISA, PCMCIA)or scanned(PCI)
 *      Resource structure is stored in the registry which will be
 *      later used by the CFG module.
 */
extern int drv_request_resources(IN u32 dw_context,
					OUT u32 *pDevNodeString);

/*
 *  ======== drv_release_resources ========
 *  Purpose:
 *      Assigns the Resources or Releases them.
 *  Parameters:
 *      dw_context:      Path to the driver Registry Key.
 *      hdrv_obj:     Handle to the Driver Object.
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 *      The Resources are released based on Bus type.
 *      Resource structure is deleted from the registry
 */
extern int drv_release_resources(IN u32 dw_context,
					struct drv_object *hdrv_obj);

int drv_request_bridge_res_dsp(void **phost_resources);

#ifdef CONFIG_BRIDGE_RECOVERY
void bridge_recover_schedule(void);
#endif
/*
 *  ======== mem_ext_phys_pool_init ========
 *  Purpose:
 *      Uses the physical memory chunk passed for internal consitent memory
 *      allocations.
 *      physical address based on the page frame address.
 *  Parameters:
 *      poolPhysBase  starting address of the physical memory pool.
 *      poolSize      size of the physical memory pool.
 *  Returns:
 *      none.
 *  Requires:
 *      - MEM initialized.
 *      - valid physical address for the base and size > 0
 */
extern void mem_ext_phys_pool_init(IN u32 poolPhysBase, IN u32 poolSize);

/*
 *  ======== mem_ext_phys_pool_release ========
 */
extern void mem_ext_phys_pool_release(void);

/*  ======== mem_alloc_phys_mem ========
 *  Purpose:
 *      Allocate physically contiguous, uncached memory
 *  Parameters:
 *      byte_size:     Number of bytes to allocate.
 *      ulAlign:    Alignment Mask.
 *      pPhysicalAddress: Physical address of allocated memory.
 *  Returns:
 *      Pointer to a block of memory;
 *      NULL if memory couldn't be allocated, or if byte_size == 0.
 *  Requires:
 *      MEM initialized.
 *  Ensures:
 *      The returned pointer, if not NULL, points to a valid memory block of
 *      the size requested.  Returned physical address refers to physical
 *      location of memory.
 */
extern void *mem_alloc_phys_mem(IN u32 byte_size,
				IN u32 ulAlign, OUT u32 *pPhysicalAddress);

/*
 *  ======== mem_flush_cache ========
 *  Purpose:
 *      Performs system cache sync with discard
 *  Parameters:
 *      pMemBuf:    Pointer to memory region to be flushed.
 *      pMemBuf:    Size of the memory region to be flushed.
 *  Returns:
 *  Requires:
 *      MEM is initialized.
 *  Ensures:
 *      Cache is synchronized
 */
extern void mem_flush_cache(void *pMemBuf, u32 byte_size, s32 FlushType);

/*
 *  ======== mem_free_phys_mem ========
 *  Purpose:
 *      Free the given block of physically contiguous memory.
 *  Parameters:
 *      pVirtualAddress:  Pointer to virtual memory region allocated
 *      by mem_alloc_phys_mem().
 *      pPhysicalAddress:  Pointer to physical memory region  allocated
 *      by mem_alloc_phys_mem().
 *      byte_size:  Size of the memory region allocated by mem_alloc_phys_mem().
 *  Returns:
 *  Requires:
 *      MEM initialized.
 *      pVirtualAddress is a valid memory address returned by
 *          mem_alloc_phys_mem()
 *  Ensures:
 *      pVirtualAddress is no longer a valid pointer to memory.
 */
extern void mem_free_phys_mem(void *pVirtualAddress,
			      u32 pPhysicalAddress, u32 byte_size);

/*
 *  ======== MEM_LINEAR_ADDRESS ========
 *  Purpose:
 *      Get the linear address corresponding to the given physical address.
 *  Parameters:
 *      pPhysAddr:  Physical address to be mapped.
 *      byte_size:     Number of bytes in physical range to map.
 *  Returns:
 *      The corresponding linear address, or NULL if unsuccessful.
 *  Requires:
 *      MEM initialized.
 *  Ensures:
 *  Notes:
 *      If valid linear address is returned, be sure to call
 *      MEM_UNMAP_LINEAR_ADDRESS().
 */
#define MEM_LINEAR_ADDRESS(pPhyAddr, byte_size) pPhyAddr

/*
 *  ======== MEM_UNMAP_LINEAR_ADDRESS ========
 *  Purpose:
 *      Unmap the linear address mapped in MEM_LINEAR_ADDRESS.
 *  Parameters:
 *      pBaseAddr: Ptr to mapped memory (as returned by MEM_LINEAR_ADDRESS()).
 *  Returns:
 *  Requires:
 *      - MEM initialized.
 *      - pBaseAddr is a valid linear address mapped in MEM_LINEAR_ADDRESS.
 *  Ensures:
 *      - pBaseAddr no longer points to a valid linear address.
 */
#define MEM_UNMAP_LINEAR_ADDRESS(pBaseAddr) {}

#endif /* DRV_ */
