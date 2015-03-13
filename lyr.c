#include <sys/types.h>
#include <sys/file.h>
#include <sys/errno.h>
#include <sys/open.h>
#include <sys/cred.h>
#include <sys/cmn_err.h>
#include <sys/modctl.h>
#include <sys/conf.h>
#include <sys/stat.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/sunldi.h>
#include<sys/cmn_err.h>
#include<sys/mtio.h>
#include <sys/uio.h>

#define	MTIOC			('m'<<8)
#define	MTIOCTOP		(MTIOC|1)	/* do a mag tape op */
#define	MTIOCGET		(MTIOC|2)	/* get tape status */
#define	MTIOCGETDRIVETYPE	(MTIOC|3)	/* get tape config data */
#define	MTIOCLTOP		(MTIOC|19)	/* do a mag tape op */
#define	MTWEOF		0	/* write an end-of-file record */
#define	MTFSF		1	/* forward space over file mark */
#define	MTBSF		2	/* backward space over file mark (1/2" only ) */
#define	MTFSR		3	/* forward space to inter-record gap */
#define	MTBSR		4	/* backward space to inter-record gap */
#define	MTREW		5	/* rewind */
#define	MTOFFL		6	/* rewind and put the drive offline */
#define	MTERASE		9	/* erase the entire tape */
#define	MTEOM		10	/* position to end of media */
#define	MTNBSF		11	/* backward space file to BOF */
#define	MTSRSZ		12	/* set record size */
#define	MTGRSZ		13	/* get record size */
#define	MTLOAD		14	/* for loading a tape (use o_delay to open 
 				 * the tape device) */
#define	MTBSSF		15	/* Backward space to x sequential filemarks */
#define	MTFSSF		16	/* Forward space to x sequential filemarks */
#define	MTTELL		17	/* get current logical block position */
#define	MTSEEK		18	/* position to logical block position */

/*#define READ_HEADER 		21
 *#define UPDATE_HEADER 	22*/

#define WRITE_HEADER 		20
#define VALIDATE 		21
#define REWIND 			22
#define NON_REWIND 		23
#define SHOW_BACKUPS 		24
#define BLK_SIZE 		512
#define PADDING_CHAR 		0x1F
#define MAGICNO_OFFSET 		0
#define LABEL_OFFSET 		10
#define DESCR_OFFSET 		31
#define CREAT_DATE_OFFSET 	132
#define LASTMOD_DATE_OFFSET 	153
#define SIZE_OFFSET 		174
#define READ_HEAD_OFFSET 	180
#define WRITE_HEAD_OFFSET 	190
#define EOR 			0x1F
#define MAX_NO_OF_FILES 	100
#define LYR_OPENED      	0x1	/* lh is valid */
#define LYR_IDENTED     	0x2	/* li is valid */
#define	MAX_REC_SIZE		262144
#define MIN_REC_SIZE 		512

typedef struct file_table {
	char            filename[20];
	int             offset;
	int             no_of_rec;
}               file_table_struct;

/*typedef struct lyr_state {
 *	ldi_handle_t    lh;
 *	ldi_ident_t     li;
 *	dev_info_t     *dip;
 *	minor_t         minor;
 *	int             flags;
 *	kmutex_t        lock;
 * 	int             write_oft;
 *	int             read_oft;
 *	int             ioctl_flag;
 *	int 		rec_size;
 *	file_table_struct files[MAX_NO_OF_FILES];
 *}               lyr_state_t;
 */

typedef struct lyr_state {
	ldi_handle_t    lh;
	ldi_ident_t     li;
	dev_info_t     *dip;
	minor_t         minor;
	int             flags;
	kmutex_t        lock;
	int             write_oft;
	int             read_oft;
	int             ioctl_flag;
	int 	rec_size;
	file_table_struct files[MAX_NO_OF_FILES];

	int WRITE_HEADER_FLAG;	//By default it's value is 0. In case of write_header operation this flag is set, after writing the header this flag is reset in the lyr_write function.
	int VALIDATE_FLAG;	//By default it's value is 0. In case of validate operation this flag is set, after reading the header this flag is reset in the lyr_read function.
	int REWIND_FLAG;		//By default it's value is 0 stating that the tape is in REWIND mode. In case user wishes NON-REWIND mode, this flag is set.

}               lyr_state_t;

static int      lyr_info(dev_info_t *, ddi_info_cmd_t, void *, void **);
static int      lyr_attach(dev_info_t *, ddi_attach_cmd_t);
static int      lyr_detach(dev_info_t *, ddi_detach_cmd_t);
static int      lyr_open(dev_t *, int, int, cred_t *);
static int      lyr_close(dev_t, int, int, cred_t *);
static int      lyr_write(dev_t, struct uio *, cred_t *);
static int      lyr_read(dev_t, struct uio *, cred_t *);
static int      lyr_ioctl(dev_t devt, int cmd, intptr_t arg, int mode, cred_t * credp, int *rval_p);
static void     *lyr_statep;
static offset_t oft;
static int      a[100], j, read_oft = 0, fileno,readfileno=0;
static int      readflag=0, closeflag, fileflag = 1;

static struct cb_ops lyr_cb_ops = {
	lyr_open,		/* open */
	lyr_close,		/* close */
	nodev,			/* strategy */
	nodev,			/* print */
	nodev,			/* dump */
	lyr_read,		/* read */
	lyr_write,		/* write */
	lyr_ioctl,		/* ioctl */
	nodev,			/* devmap */
	nodev,			/* mmap */
	nodev,			/* segmap */
	nochpoll,		/* poll */
	ddi_prop_op,		/* prop_op */
	NULL,			/* streamtab */
	D_NEW | D_MP,		/* cb_flag */
	CB_REV,			/* cb_rev */
	nodev,			/* aread */
	nodev			/* awrite */
};

static struct dev_ops lyr_dev_ops = {
	DEVO_REV,		/* devo_rev, */
	0,			/* refcnt */
	lyr_info,		/* getinfo */
	nulldev,		/* identify */
	nulldev,		/* probe */
	lyr_attach,		/* attach */
	lyr_detach,		/* detach */
	nodev,			/* reset */
	&lyr_cb_ops,		/* cb_ops */
	NULL,			/* bus_ops */
	NULL			/* power */
};

static struct modldrv modldrv = {
	&mod_driverops,
	"LDI example driver",
	&lyr_dev_ops
};

static struct modlinkage modlinkage = {
	MODREV_1,
	&modldrv,
	NULL
};

int
_init(void)
{
	int rv;

	cmn_err(CE_NOTE, "Inside init");

	if ((rv = ddi_soft_state_init(&lyr_statep, sizeof(lyr_state_t),
				      0)) != 0) {
		cmn_err(CE_WARN, "lyr _init: soft state init failed\n");
		return (rv);
	}
	if ((rv = mod_install(&modlinkage)) != 0) {
		cmn_err(CE_WARN, "lyr _init: mod_install failed\n");
		goto FAIL;
	}
	return (rv);
	/* NOTEREACHED */
FAIL:
	ddi_soft_state_fini(&lyr_statep);
	return (rv);
}

int
_info(struct modinfo * modinfop)
{
	cmn_err(CE_NOTE, "Inside info");
	return (mod_info(&modlinkage, modinfop));
}

int
_fini(void)
{
	int             rv;
	cmn_err(CE_NOTE, "Inside fini");
	if ((rv = mod_remove(&modlinkage)) != 0) {
		return (rv);
	}
	ddi_soft_state_fini(&lyr_statep);
	return (rv);
}

/*
 * 1:1 mapping between minor number and instance
 */

static int
lyr_info(dev_info_t * dip, ddi_info_cmd_t infocmd, void *arg, void **result)
{
	int             inst;
	minor_t         minor;
	lyr_state_t    *statep;
	char           *myname = "lyr_info";
	minor = getminor((dev_t) arg);
	inst = minor;
	cmn_err(CE_NOTE, "Inside lyr_info");
	switch (infocmd) {
	case DDI_INFO_DEVT2DEVINFO:
		statep = ddi_get_soft_state(lyr_statep, inst);
		if (statep == NULL) {
			cmn_err(CE_WARN, "%s: get soft state "
				"failed on inst %d\n", myname, inst);
			return (DDI_FAILURE);
		}
		*result = (void *) statep->dip;
		break;
	case DDI_INFO_DEVT2INSTANCE:
		*result = (void *) inst;
		break;
	default:
		break;
	}
	return (DDI_SUCCESS);
}

static int
lyr_attach(dev_info_t * dip, ddi_attach_cmd_t cmd)
{
	int             inst;
	lyr_state_t    *statep;
	char           *myname = "lyr_attach";
	cmn_err(CE_NOTE, "Inside lyr_attach");
	switch (cmd) {
	case DDI_ATTACH:
		inst = ddi_get_instance(dip);
		if (ddi_soft_state_zalloc(lyr_statep, inst) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "%s: ddi_soft_state_zallac failed "
				"on inst %d\n", myname, inst);
			goto FAIL;
		}
		statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
		//cmn_err(CE_NOTE, "%s %d %u", statep->lh, statep->lh, statep->lh);
		//cmn_err(CE_NOTE, "dev ptr:%u", statep->dip);
		if (statep == NULL) {
			cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
				"inst %d\n", myname, inst);
			goto FAIL;
		}
		statep->dip = dip;
		statep->minor = inst;
		statep->rec_size=MIN_REC_SIZE;
		if (ddi_create_minor_node(dip, "node", S_IFCHR, statep->minor,
					  DDI_PSEUDO, 0) != DDI_SUCCESS) 
		{
			cmn_err(CE_WARN, "%s: ddi_create_minor_node failed on "
				"inst %d\n", myname, inst);
			goto FAIL;
		}

		if (ddi_create_minor_node(dip,"rnode", S_IFCHR, statep->minor,DDI_PSEUDO,0) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "%s: ddi_create_minor_node failed on "
				"inst %d\n", myname, inst);
			goto FAIL;
		}

		mutex_init(&statep->lock, NULL, MUTEX_DRIVER, NULL);
		return (DDI_SUCCESS);
	case DDI_RESUME:
	case DDI_PM_RESUME:
	default:
		break;
	}
	return (DDI_FAILURE);
	/* NOTREACHED */
FAIL:
	ddi_soft_state_free(lyr_statep, inst);
	ddi_remove_minor_node(dip, NULL);
	return (DDI_FAILURE);
}

static int
lyr_detach(dev_info_t * dip, ddi_detach_cmd_t cmd)
{
	int             inst;
	lyr_state_t    *statep;
	char           *myname = "lyr_detach";
	inst = ddi_get_instance(dip);
	statep = ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_detach");
	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: get soft state failed on "
			"inst %d\n", myname, inst);
		return (DDI_FAILURE);
	}
	if (statep->dip != dip) {
		cmn_err(CE_WARN, "%s: soft state does not match devinfo "
			"on inst %d\n", myname, inst);
		return (DDI_FAILURE);
	}
	switch (cmd) {
	case DDI_DETACH:
		mutex_destroy(&statep->lock);
		ddi_soft_state_free(lyr_statep, inst);
		ddi_remove_minor_node(dip, NULL);
		return (DDI_SUCCESS);
	case DDI_SUSPEND:
	case DDI_PM_SUSPEND:
	default:
		break;
	}
	return (DDI_FAILURE);
}

/*
 * on this driver’s open, we open the target specified by a property and
 * store the layered handle and ident in our soft state. a good target would
 * be "/dev/console" or more interestingly, a pseudo terminal as specified by
 * the tty command
 */
/* ARGSUSED */

static int
lyr_open(dev_t * devtp, int oflag, int otyp, cred_t * credp)
{
	int             rv, inst = getminor(*devtp);
	lyr_state_t    *statep;
	char           *myname = "lyr_open";
	dev_info_t     *dip;
	char           *lyr_targ = NULL;
	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_open");
	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
	dip = statep->dip;
	/*
	 * our target device to open should be specified by the "lyr_targ"
	 * string property, which should be set in this driver’s .conf file
	 */
	if (ddi_prop_lookup_string(DDI_DEV_T_ANY, dip, DDI_PROP_NOTPROM,
			       "lyr_targ", &lyr_targ) != DDI_PROP_SUCCESS) {
		cmn_err(CE_WARN, "%s: ddi_prop_lookup_string failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
	/*
	 * since we only have one pair of lh’s and li’s available, we
	 * don’t allow multiple on the same instance
	 */
	mutex_enter(&statep->lock);
	if (statep->flags & (LYR_OPENED | LYR_IDENTED)) {
		cmn_err(CE_WARN, "%s: multiple layered opens or idents "
			"from inst %d not allowed\n", myname, inst);
		mutex_exit(&statep->lock);
		ddi_prop_free(lyr_targ);
		return (EIO);
	}
	rv = ldi_ident_from_dev(*devtp, &statep->li);
	if (rv != 0) {
		cmn_err(CE_WARN, "%s: ldi_ident_from_dev failed on inst %d\n",
			myname, inst);
		goto FAIL;
	}
	statep->flags |= LYR_IDENTED;
	rv = ldi_open_by_name(lyr_targ, FREAD | FWRITE, credp, &statep->lh,
			      statep->li);
	if (rv != 0) {
		cmn_err(CE_WARN, "%s: ldi_open_by_name failed on inst %d\n",
			myname, inst);
		goto FAIL;
	}
	statep->flags |= LYR_OPENED;
	cmn_err(CE_CONT, "\n%s: opened target \E263\99%s\E263\99 successfully on inst %d\n",
		myname, lyr_targ, inst);
	rv = 0;
FAIL:
	/* cleanup on error */
	if (rv != 0) {
		if (statep->flags & LYR_OPENED)
			(void) ldi_close(statep->lh, FREAD | FWRITE, credp);
		if (statep->flags & LYR_IDENTED)
			ldi_ident_release(statep->li);
		statep->flags &= ~(LYR_OPENED | LYR_IDENTED);
	}
	statep->ioctl_flag = 0;
	mutex_exit(&statep->lock);
	if (lyr_targ != NULL)
		ddi_prop_free(lyr_targ);
	return (rv);
}

/*
 * on this driver’s close, we close the target indicated by the lh member
 * in our soft state and release the ident, li as well. in fact, we MUST do
 * both of these at all times even if close yields an error because the
 * device framework effectively closes the device, releasing all data
 * associated with it and simply returning whatever value the target’s
 * close(9E) returned. therefore, we must as well.
 */
/* ARGSUSED */

static int
lyr_close(dev_t devt, int oflag, int otyp, cred_t * credp)
{
	int             rv, inst = getminor(devt);
	lyr_state_t    *statep;
	char           *myname = "lyr_close";
	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_close");
	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
	mutex_enter(&statep->lock);
	rv = ldi_close(statep->lh, FREAD | FWRITE, credp);
	if (rv != 0) {
		cmn_err(CE_WARN, "%s: ldi_close failed on inst %d, but will ",
			"continue to release ident\n", myname, inst);
	}
	ldi_ident_release(statep->li);
	if (rv == 0) {
		cmn_err(CE_CONT, "\n%s: closed target successfully on "
			"inst %d\n", myname, inst);
	}
	if (closeflag) {
		statep->files[fileno].no_of_rec = j;
		cmn_err(CE_NOTE, "filebuf : %s fileoft:%d fileno:%d NO_OF_REC:%d	", statep->files[fileno].filename, statep->files[fileno].offset, fileno, statep->files[fileno].no_of_rec);
		fileno++;
		j = 0;
		closeflag = 0;
		fileflag = 1;
	}
		if(readflag)
		{
                    	readflag=0;
                        
                        cmn_err(CE_NOTE,"rfileno:%d",readfileno);
                        readfileno++;
                 }
	statep->flags &= ~(LYR_OPENED | LYR_IDENTED);
	mutex_exit(&statep->lock);
	return (rv);
}


/* ARGSUSED */
/*static int
lyr_write(dev_t devt, struct uio * uiop, cred_t * credp)
{
	int             rv, i = 0, n = 1, inst = getminor(devt);
	lyr_state_t    *statep;
	char           *myname = "lyr_write";
	char            buf[512];
	uint64_t size;

	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_write");

	if (statep == NULL) 
	{
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
       
	if (statep->WRITE_HEADER_FLAG == 1) 
	{
		ldi_get_size(statep->lh,&size);
		//statep->read_oft = statep->write_oft = size - 1024;
		uiop->uio_offset= size - 1024;
		rv = copyin(uiop->uio_iov->iov_base, buf, 512);
		
		//cmn_err(CE_NOTE, "buffferr %s,RV=%d", buf, rv);
		//cmn_err(CE_NOTE, "first: read_oft=%d write_oft=%d", statep->read_oft, statep->write_oft);
		
		rv = ldi_write(statep->lh, uiop, credp);
		statep->WRITE_HEADER_FLAG = 0;
		return rv;
	} 
	
/*	else if (statep->ioctl_flag == UPDATE_HEADER) {
 *		copyin(uiop->uio_iov->iov_base, buf, 512);
 *		buf[WRITE_HEAD_OFFSET] = 5;
 *		copyout(buf, uiop->uio_iov->iov_base, 512);
 *		rv = ldi_write(statep->lh, uiop, credp);
 *		cmn_err(CE_NOTE, " buf in update header%s", buf);
 *		statep->ioctl_flag = 0;
 *	}
 */

/*	else 
	{
		oft = statep->write_oft;
		uiop->uio_offset = oft * 512;
		i = uiop->uio_resid / 512;

		cmn_err(CE_NOTE, "b4: off%d	 res%d", uiop->uio_offset, uiop->uio_resid);
		j++;

		if (fileflag) 
		{
			copyin(uiop->uio_iov->iov_base, statep->files[fileno].filename, 20);

			fileflag = 0;
			statep->files[fileno].offset = oft;
			closeflag = 1;
			//cmn_err(CE_NOTE, "filebuf : %s fileoft:%d fileno:%d	", statep->files[fileno].filename, statep->files[fileno].offset, fileno);
		}
		cmn_err(CE_NOTE, " ARAAYYYYY b4 IOVEC buff:%s off%d	res%d   j:%d a[fileno]%d",uiop->uio_iov->iov_base,uiop->uio_offset, uiop->uio_resid, j, a[fileno]);

		rv = ldi_write(statep->lh, uiop, credp);

		oft++;
		oft += i;
		cmn_err(CE_NOTE, " ARAAYYYYY after off%d	res%d   j:%d a[fileno]%d", uiop->uio_offset, uiop->uio_resid, j, a[n]);
		/*
		 * if(rv==0) { cmn_err(CE_NOTE,"hello"); closeflag=1;
		 * a[fileno]=j;}
		 */
/*		statep->write_oft = oft;
		cmn_err(CE_NOTE, " ARAAYYYYY after IOVEC buff:%s off%d	res%d   j:%d a[fileno]%d",uiop->uio_iov->iov_base,uiop->uio_offset, uiop->uio_resid, j, a[fileno]);

	}
}
	/*
	 * else { cmn_err(CE_NOTE,"b4 eveery write :read oft %d write_oft=%d
	 * stae->iflag=%d",statep->read_oft,statep->write_oft,statep->ioctl_fl
	 * ag);
	 * 
	 * uiop->uio_offset=(statep->write_oft)*512;  i=uiop->uio_resid/512;
	 * rv=copyin(uiop->uio_iov->iov_base,buf,512);
	 * len=uiop->uio_iov->iov_len; buf[len-2]=EOR; len++;
	 * cmn_err(CE_NOTE,"bufferrrrrrrrr %s",buf);
	 * uiop->uio_iov->iov_len=len; uiop->uio_resid=len;
	 * copyout(buf,uiop->uio_iov->iov_base,sizeof(buf));
	 * cmn_err(CE_NOTE,"bufferrrrrrrrr %s",uiop->uio_iov->iov_base); rv=
	 * ldi_write(statep->lh, uiop, credp); (statep->write_oft)++;
	 * (statep->write_oft)+=i; cmn_err(CE_NOTE,"kjkljrv of uread%d eveery
	 * write :write_oft=%d
	 * stae->iflag=%d",rv,statep->write_oft,statep->ioctl_flag);
	 * 
	 * 
	 * 
	 * } 
	 *
	 *cmn_err(CE_NOTE, "after eveery write :read_oft=%d write_oft=%d", statep->read_oft, statep->write_oft);
	 *cmn_err(CE_NOTE, "%d    %d %d", uiop->uio_resid, uiop->uio_offset, oft);
	 *return rv;
}
*/
 
static int
lyr_read(dev_t devt, struct uio * uiop, cred_t * credp)
{
	int             resid=0,rv, inst = getminor(devt);
	lyr_state_t    *statep;
	char           *myname = "lyr_read";
	uint64_t size;

	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_read");
	cmn_err(CE_NOTE, "read uio buffer:%s", uiop->uio_iov->iov_base);
	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
	/*
	 * while(uiop->uio_offset!=193*512) { uiop->uio_offset=192*512;
	 * uiop->uio_resid=512;
	 * 
	 * 
	 * rv=ldi_read(statep->lh, uiop, credp); }
	 */

	//if(readfileno>=fileno) return(EIO);	

	if(statep->VALIDATE_FLAG==1) 
	{ 
		ldi_get_size(statep->lh,&size);
		while(uiop->uio_offset< (size - 512))
		{
		uiop->uio_offset= size - 1024;
                uiop->uio_resid=512;	
		cmn_err(CE_NOTE,"inside validate size: %d",size); 
	 	rv=ldi_read(statep->lh, uiop, credp);
                
		cmn_err(CE_NOTE,"after ldi_read in validate ret val:%d ofset:%d resid:%d",rv,uiop->uio_offset,uiop->uio_resid);
                statep->VALIDATE_FLAG= 0; 
		}
	}
	 
	else 
	{
        	resid=uiop->uio_resid;
		uiop->uio_offset = read_oft;
		rv = ldi_read(statep->lh, uiop, credp);
		read_oft += (resid+512);
	}

	if(rv==0)
	readflag=1;
	return rv;
}


/* ARGSUSED */
static int
lyr_write(dev_t devt, struct uio * uiop, cred_t * credp)
{
	int             rv, i = 0, n = 1, inst = getminor(devt);
	lyr_state_t    *statep;
	char           *myname = "lyr_write";
	char            buf[512];
	uint64_t size;

	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_write");

	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
       
	if (statep->WRITE_HEADER_FLAG == 1) 
	{
		ldi_get_size(statep->lh,&size);
		//statep->read_oft = statep->write_oft = size - 1024;
		uiop->uio_offset= size - 1024;
		rv = copyin(uiop->uio_iov->iov_base, buf, 512);
		
		//cmn_err(CE_NOTE, "buffferr %s,RV=%d", buf, rv);
		//cmn_err(CE_NOTE, "first: read_oft=%d write_oft=%d", statep->read_oft, statep->write_oft);
		
		rv = ldi_write(statep->lh, uiop, credp);
		statep->WRITE_HEADER_FLAG = 0;
		return rv;
	} 
	
/*	else if (statep->ioctl_flag == UPDATE_HEADER) {
		copyin(uiop->uio_iov->iov_base, buf, 512);
		buf[WRITE_HEAD_OFFSET] = 5;
		copyout(buf, uiop->uio_iov->iov_base, 512);
		rv = ldi_write(statep->lh, uiop, credp);
		cmn_err(CE_NOTE, " buf in update header%s", buf);
		statep->ioctl_flag = 0;
	}
*/

	 else {
		oft = statep->write_oft;
		uiop->uio_offset = oft * 512;
		i = uiop->uio_resid / 512;

		cmn_err(CE_NOTE, "b4: off%d	 res%d", uiop->uio_offset, uiop->uio_resid);
		j++;

		if (fileflag) {
			copyin(uiop->uio_iov->iov_base, statep->files[fileno].filename, 20);

			fileflag = 0;
			statep->files[fileno].offset = oft;
			closeflag = 1;
			//cmn_err(CE_NOTE, "filebuf : %s fileoft:%d fileno:%d	", statep->files[fileno].filename, statep->files[fileno].offset, fileno);
		}
		cmn_err(CE_NOTE, " ARAAYYYYY b4 IOVEC buff:%s off%d	res%d   j:%d a[fileno]%d",uiop->uio_iov->iov_base,uiop->uio_offset, uiop->uio_resid, j, a[fileno]);

		rv = ldi_write(statep->lh, uiop, credp);

		oft++;
		oft += i;
		cmn_err(CE_NOTE, " ARAAYYYYY after off%d	res%d   j:%d a[fileno]%d", uiop->uio_offset, uiop->uio_resid, j, a[n]);
		/*
		 * if(rv==0) { cmn_err(CE_NOTE,"hello"); closeflag=1;
		 * a[fileno]=j;}
		 */
		statep->write_oft = oft;
		cmn_err(CE_NOTE, " ARAAYYYYY after IOVEC buff:%s off%d	res%d   j:%d a[fileno]%d",uiop->uio_iov->iov_base,uiop->uio_offset, uiop->uio_resid, j, a[fileno]);

	}

	/*
	 * else { cmn_err(CE_NOTE,"b4 eveery write :read oft %d write_oft=%d
	 * stae->iflag=%d",statep->read_oft,statep->write_oft,statep->ioctl_fl
	 * ag);
	 * 
	 * uiop->uio_offset=(statep->write_oft)*512;  i=uiop->uio_resid/512;
	 * rv=copyin(uiop->uio_iov->iov_base,buf,512);
	 * len=uiop->uio_iov->iov_len; buf[len-2]=EOR; len++;
	 * cmn_err(CE_NOTE,"bufferrrrrrrrr %s",buf);
	 * uiop->uio_iov->iov_len=len; uiop->uio_resid=len;
	 * copyout(buf,uiop->uio_iov->iov_base,sizeof(buf));
	 * cmn_err(CE_NOTE,"bufferrrrrrrrr %s",uiop->uio_iov->iov_base); rv=
	 * ldi_write(statep->lh, uiop, credp); (statep->write_oft)++;
	 * (statep->write_oft)+=i; cmn_err(CE_NOTE,"kjkljrv of uread%d eveery
	 * write :write_oft=%d
	 * stae->iflag=%d",rv,statep->write_oft,statep->ioctl_flag);
	 * 
	 * 
	 * 
	 * } */



	//cmn_err(CE_NOTE, "after eveery write :read_oft=%d write_oft=%d", statep->read_oft, statep->write_oft);

	//cmn_err(CE_NOTE, "%d    %d %d", uiop->uio_resid, uiop->uio_offset, oft);
	return rv;
}

 
/*static int
lyr_read(dev_t devt, struct uio * uiop, cred_t * credp)
{
	int             resid=0,rv, inst = getminor(devt);
	lyr_state_t    *statep;
	char           *myname = "lyr_read";
	uint64_t size;

	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	cmn_err(CE_NOTE, "Inside lyr_read");
	cmn_err(CE_NOTE, "read uio buffer:%s", uiop->uio_iov->iov_base);
	if (statep == NULL) {
		cmn_err(CE_WARN, "%s: ddi_get_soft_state failed on "
			"inst %d\n", myname, inst);
		return (EIO);
	}
	/*
	 * while(uiop->uio_offset!=193*512) { uiop->uio_offset=192*512;
	 * uiop->uio_resid=512;
	 * 
	 * 
	 * rv=ldi_read(statep->lh, uiop, credp); }
	 */

/*	if(readfileno>=fileno) return(EIO);	

	 if(statep->VALIDATE_FLAG==1) 
	{ 
		ldi_get_size(statep->lh,&size);
		while(uiop->uio_offset< (size - 512))
		{
		uiop->uio_offset= size - 1024;
                uiop->uio_resid=512;	
		cmn_err(CE_NOTE,"inside validate size: %d",size); 
	 	rv=ldi_read(statep->lh, uiop, credp);
                
		cmn_err(CE_NOTE,"after ldi_read in validate ret val:%d ofset:%d resid:%d",rv,uiop->uio_offset,uiop->uio_resid);
                statep->VALIDATE_FLAG= 0; 
		}
	}
	 
	 else 
	{
        		resid=uiop->uio_resid;
		uiop->uio_offset = read_oft;
		rv = ldi_read(statep->lh, uiop, credp);
		read_oft += (resid+512);
	}

	if(rv==0)
	readflag=1;
	return rv;
}

*/
static int
lyr_ioctl(dev_t devt, int cmd, intptr_t arg, int mode,
	  cred_t * credp, int *rval_p)
{

	int             options,rv=0,i=0,count=0,inst = getminor(devt);
	lyr_state_t    *statep;
	struct mtlop lyrmt;

        // structure for MTIOCGET - mag tape get status command
	struct mtget lyrmtget;
	// struct for MTIOCGETDRIVETYPE - get tape config data
     	struct mtdrivetype lyrmtconfig;
	struct iovec	iov;
    	struct uio	uio,*uiop;

	statep = (lyr_state_t *) ddi_get_soft_state(lyr_statep, inst);
	if (statep == NULL) 
	{
		cmn_err(CE_WARN, "lyr_ioctl: ddi_get_soft_state failed on "
			"inst %d\n", inst);
		return (EIO);
	}
	cmn_err(CE_NOTE, "cmd:%ccmd:%d arg:%d ",cmd,cmd,  arg);
	
	switch(cmd)
	{
		case WRITE_HEADER:
			cmn_err(CE_NOTE,"inside ioctl write header!!");
			statep->WRITE_HEADER_FLAG = 1;
			break;
		case VALIDATE:
			statep->VALIDATE_FLAG = 1;
			break;
		case REWIND:
			statep->REWIND_FLAG = 0;
			cmn_err(CE_NOTE,"inside rewind ioctl!!");
			break;
		case NON_REWIND:
			statep->REWIND_FLAG = 1;
			cmn_err(CE_NOTE,"inside non rewind ioctl!!");
			break;
		case MTIOCLTOP: 
			cmn_err(CE_NOTE, " hiiiiiiiiiiiiiiiiiiiii ");
			ddi_copyin((void *)arg, &lyrmt, sizeof (struct mtlop),mode);
			options=lyrmt.mt_op;
			count=lyrmt.mt_count;
			cmn_err(CE_NOTE, "cmd:%d count %d", lyrmt.mt_op,lyrmt.mt_count);
			switch (options) 
			{
					/*case UPDATE_HEADER:
					statep->ioctl_flag = UPDATE_HEADER;
					break;*/
				case MTFSF:
			                readfileno+=count;
			                if(readfileno<=fileno)
			                {
              					read_oft=(statep->files[readfileno].offset)*512;
              				}
			      		cmn_err(CE_NOTE, "hello i m in ioctl fsf:read_oft %d readfile no:%d",read_oft,readfileno);
					break;
				case MTBSF:
			                readfileno-=count;
			                if(readfileno>=0)
			                {
                                        	read_oft=(statep->files[readfileno].offset)*512;
			                }
					cmn_err(CE_NOTE, "hello i m in ioctl read_oft:%d bsfreadfieno:%d",read_oft,readfileno);
					break;
				case MTFSR:
					cmn_err(CE_NOTE, "hello i m in ioctl fsr");
					break;
				case MTBSR:
					cmn_err(CE_NOTE, "hello i m in ioctl bsr");
					break;
			       	case MTREW:
					cmn_err(CE_NOTE, "hello i m in ioctl rewind");
			                read_oft=oft=statep->write_oft=0;
					break;
			        case MTERASE:
					read_oft=oft=statep->write_oft=0;
                			for(i=0;i<fileno;i++)
					{
						copyout(NULL,statep->files[fileno].filename,20);
						statep->files[fileno].offset=0;
						statep->files[fileno].no_of_rec=0;
			                }
		                        fileno=0;
			         	readfileno=0;
		 			cmn_err(CE_NOTE, "hello i m in ioctl erase");
					break;
				case MTTELL:
			                lyrmt.mt_op=MTTELL;
					lyrmt.mt_count=oft;
				        ddi_copyout( &lyrmt,(void *)arg, sizeof (struct mtlop),mode);
			                cmn_err(CE_NOTE, "hello i m in ioctl erase oft:%d",oft);
			                break;
			        case MTSEEK:
					statep->write_oft=oft=count;
					read_oft=count*512;
					lyrmt.mt_op=MTTELL;
					ddi_copyout( &lyrmt,(void *)arg, sizeof (struct mtlop),mode);
			                cmn_err(CE_NOTE, "hello i m in ioctl mtseek");
		               		break;
				case MTSRSZ:
			    		if (count < MIN_REC_SIZE) 
					{
    						return (EINVAL);
    					}
			    		if (count > MAX_REC_SIZE) 
					{
    						return (EINVAL);
   					}
			   		if( count % MIN_REC_SIZE != 0)
					{
						return(EINVAL); 
					}
			  		statep->rec_size = count;
			  		return (0);
					break;
			 	case MTGRSZ:
 			  		lyrmt.mt_count = statep->rec_size;
   					ddi_copyout( &lyrmt,(void *)arg, sizeof (struct mtlop),mode);
    					return (0);
				    	break;
				default:
					cmn_err(CE_NOTE, "hello i m in ioctl default");
			}
			break;
         	case MTIOCGET:
			cmn_err(CE_NOTE, "hello i m in status");
			ddi_copyin((void *)arg, &lyrmtget, sizeof (struct mtget),mode);
			lyrmtget.mt_type=0;	/* type of magtape device */

			/* the following two registers are grossly device dependent */
		    	lyrmtget.mt_dsreg=0;	/* ``drive status'' register */
     			lyrmtget.mt_erreg=0;	/* ``error'' register */

			/* optional error info. */
		     	lyrmtget.mt_resid=0;	/* residual count */
    			lyrmtget.mt_fileno=0;	/* file number of current position */
    			lyrmtget.mt_blkno=oft;	/* block number of current position */
    			lyrmtget.mt_flags=0;
    			lyrmtget.mt_bf=0;	/* optimum blocking factor */
			ddi_copyout( &lyrmtget,(void *)arg, sizeof (struct mtget),mode);
			break;
		case MTIOCGETDRIVETYPE:
			cmn_err(CE_NOTE, "hello i m in config");
			(void)snprintf(lyrmtconfig.name,13,"%s","virtual tape");	/* Name, for debug */
    			(void)snprintf(lyrmtconfig.vid,7,"%s","krcube")	;		/* Vendor id and model (product) id */
   		 	lyrmtconfig.type='V';						/* Drive type for driver */
   		 	lyrmtconfig.bsize=512;						/* Block size */
   		 	lyrmtconfig.options=2;						/* Drive options */
   		 	lyrmtconfig.max_rretries=1;					/* Max read retries */
   		 	lyrmtconfig.max_wretries=1;					/* Max write retries */
   		 	(void)snprintf(lyrmtconfig.densities,5,"%s","high");		/* density codes, low->hi */
  			lyrmtconfig.default_density='m';				/* Default density chosen */
			(void)snprintf(lyrmtconfig.speeds,5,"%s","high");		/* speed codes, low->hi */
			lyrmtconfig.non_motion_timeout=60;  				/* Inquiry type commands */
			lyrmtconfig.io_timeout=60;					/* io timeout. seconds */
			lyrmtconfig.rewind_timeout=60;					/* rewind timeout. seconds */
			lyrmtconfig.space_timeout=60;					/* space cmd timeout. seconds */
			lyrmtconfig.load_timeout=60;					/* load tape time in seconds */
			lyrmtconfig.unload_timeout=60;					/* Unload tape time in scounds */
			lyrmtconfig.erase_timeout=60;		
			ddi_copyout( &lyrmtconfig,(void *)arg, sizeof (struct mtdrivetype),mode);
			break;
		default:
			cmn_err(CE_NOTE, "hello i m in default default");
	}
	return (*rval_p);
}





/*path=kmem_alloc(8192, KM_SLEEP);
 *void) snprintf(path, 10, "%s", myname);
 *uiop=&uio;
 *bzero(uiop, sizeof(struct uio));
 *iov.iov_base = (caddr_t)arg;
 *uio.uio_iov = &iov;
 *uio.uio_iovcnt = 1;
 *uio.uio_segflg = UIO_USERSPACE;
 *uio.uio_loffset = 0;
 *uio.uio_resid =8192;
 *uio.uio_fmode = FREAD;
 *uio.uio_llimit = 0x7777777;
 *cmn_err(CE_NOTE,"11 read:resid:%d offset:%d rv%d  ",uio.uio_resid,uio.uio_loffset,rv);
 *rv= ldi_read(statep->lh, uiop, credp);
 *copyout(iov.iov_base,path,512);
 *cmn_err(CE_NOTE,"22read:resid:%d offset:%d rv%d  uio buf:%s",uio.uio_resid,uio.uio_loffset,rv,iov.iov_base);
 */
