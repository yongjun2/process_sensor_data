#include "includes.h"



volatile uint8_t bad_block_table[NBR_BLOCK];

/*! \brief Spare area description for pages with 16 bytes OOB. */
static struct nand_out_of_bounce_layout nand_oob_16 = {
	.size		= 16,
	.eccbytes	= 6,
	.eccpos		= {
		0, 1, 2, 3, 6, 7,
	},
};

/*! \brief Spare area description for pages with 64 bytes OOB. */
static struct nand_out_of_bounce_layout nand_oob_64 = {
	.size		= 64,
	.eccbytes	= 24,
	.eccpos		= {
		1, 2, 3, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 13, 14, 15, 16,
		17, 18, 19, 20, 21, 22, 23, 24,
	},
};

static int32_t nand_gpio_reset(struct nand_driver_data *nfd);

static int32_t nand_gpio_command_failed(struct nand_driver_data *nfd);

static void nand_gpio_write_addr(const uint8_t addr);

static int32_t nand_gpio_read_id(struct nand_driver_data *nfd);

void nand_gpio_block_markok(struct nand_driver_data *nfd,
		const uint32_t block);
#ifdef NAND_MARK_BLOCKS_OK
static int32_t nand_gpio_block_setok(struct nand_driver_data *nfd,
		const uint32_t block);
#endif
void nand_gpio_block_markbad(struct nand_driver_data *nfd,
		const uint32_t block);
#ifdef NAND_MARK_BLOCKS_BAD
static int32_t nand_gpio_block_setbad(struct nand_driver_data *nfd,
		const uint32_t block);
#endif
static int32_t nand_gpio_block_checkbad(struct nand_driver_data *nfd,
		const uint32_t block);
static int32_t nand_gpio_block_isbad(struct nand_driver_data *nfd,
		const uint32_t block);

#if NAND_ECC_TYPE == NAND_ECC_SW
static int32_t nand_gpio_read_oob_ecc(struct nand_driver_data *nfd,
		uint64_t *ecc, const uint32_t block,
		const uint32_t offset);
static int32_t nand_gpio_write_oob_ecc(struct nand_driver_data *nfd,
		const uint64_t ecc, const uint32_t block,
		const uint32_t offset);
#endif

/*! \brief Wait for NAND flash R/B signal to go ready.
 *
 *  This function will wait for the R/B signal to go to the ready state. It
 *  will do a polled wait with the possibility for a timeout.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *
 *  \return 0 on success, an error number elsewise.
 */


static int32_t nand_gpio_wait_ready(struct nand_driver_data *nfd)
{
	//porting - end
	/* Should be done within 3 milliseconds for all commands. */
	volatile uint64_t timeout = 3000000;  //0x200000; //approx. 3secs

	while (timeout > 0) {
		//if (gpio_get_pin_value(nfd->gpio_cont_pin, nfd->gpio_rb)) return 0;
		if (gpio_get_rb_pin_value())
			return 0;
		--timeout;
		nDelay(0);
	}

	return -ETIMEDOUT;
}

/*! \brief Write a command to the NAND flash device.
 *
 *  This function will write a command to the NAND flash device, it is used
 *  by the other functions internally in the NAND GPIO driver.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param command Command to write to the NAND flash device.
 */
static void _nand_gpio_write_cmd(struct nand_driver_data *nfd,
		const uint8_t command)
{
//	_nand_gpio_write_io(nfd, command);
//	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_cle);
//	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_we);

	//_nand_gpio_write_io(command);
	gpio_cle_set();
	gpio_we_reset();
	_nand_gpio_write_io(command);
	//_nand_gpio_write_io(command);
	//_delay_ns(t_wp); //porting - end.
	//nDelay(t_wp);

	//gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_we);
	gpio_we_set();

	//_delay_ns(t_alh); //porting - end.
	//nDelay(t_alh);

	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_cle);
	//gpio_we_reset();
	gpio_cle_reset();

}

/*! \brief Write a command to the NAND flash device.
 *
 *  This function will write a command to the NAND flash device, it is used
 *  by the other functions internally in the NAND GPIO driver.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param command Command to write to the NAND flash device.
 *  \param addr Address part of the command if appropriate.
 *  \param offset Offset part of the address if appropriate.
 *
 *  \return 0 on success, an error number elsewise.
 */
static int32_t nand_gpio_write_cmd(struct nand_driver_data *nfd,
		const uint8_t command, const uint32_t addr,
		const uint32_t offset)
{
	int32_t retval = 0;

	switch (command) {
	case NAND_CMD_RESET:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_RESET);
		retval = nand_gpio_wait_ready(nfd);
		return retval;
	case NAND_CMD_ID:

		gpio_ale_reset();
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_READID);
//		gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
//		nand_gpio_write_addr(nfd, 0);
//		gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
		gpio_ale_set();
		nand_gpio_write_addr(0);
		gpio_ale_reset();

		return retval;
	case NAND_CMD_STATUS:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_STATUS);
		return retval;
	case NAND_CMD_READ:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_READ1);
		break;
	case NAND_CMD_RANDOM_READ:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_RANDOM_READ1);
		break;
	case NAND_CMD_PROGRAM:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_PAGEPROG1);
		//nDelay(t_adl);
		break;
	case NAND_CMD_RANDOM_PROGRAM:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_RANDOM_PAGEPROG);
		break;
	case NAND_CMD_PROGRAM_COMPLETE:
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_PAGEPROG2);
		retval = nand_gpio_wait_ready(nfd);
		return retval;
	case NAND_CMD_ERASE:
//		_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE1);
//		gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
//		nand_gpio_write_addr(nfd, (addr)       & 0xff);
//		nand_gpio_write_addr(nfd, (addr >> 8)  & 0xff);
//		nand_gpio_write_addr(nfd, (addr >> 16) & 0xff);
//		gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
//		_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE2);
//		retval = nand_gpio_wait_ready(nfd);


		_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE1);

		gpio_ale_set();
		nand_gpio_write_addr((addr)       & 0xff);
		nand_gpio_write_addr((addr >> 8)  & 0xff);
		nand_gpio_write_addr((addr >> 16) & 0xff);
		gpio_ale_reset();
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE2);
		retval = nand_gpio_wait_ready(nfd);
		return retval;
	default:
		return -EINVAL;
	}

	/* From here the command is either
	 * NAND_CMD_READ, NAND_CMD_RANDOM_READ or NAND_CMD_PROGRAM.
	 */
	//gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
	gpio_ale_set();
	//nDelay(t_adl);

	nand_gpio_write_addr((offset)      & 0xff);
	nand_gpio_write_addr((offset >> 8) & 0xff);

	if (command == NAND_CMD_READ || command == NAND_CMD_PROGRAM)
	{
		nand_gpio_write_addr((addr)       & 0xff);
		nand_gpio_write_addr((addr >> 8)  & 0xff);
		nand_gpio_write_addr((addr >> 16) & 0xff);
	}
	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
	//nDelay(t_adl);
	gpio_ale_reset();

	if (command == NAND_CMD_READ)
	{
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_READ2);
	}
	else if (command == NAND_CMD_RANDOM_READ)
	{
		_nand_gpio_write_cmd(nfd, NAND_COMMAND_RANDOM_READ2);
	}
	else
	{
		return retval;
	}

	return nand_gpio_wait_ready(nfd);
}

/*! \brief Write an address to the NAND flash device.
 *
 *  This function will write an address to the NAND flash device, it is used
 *  by the other functions internally in the NAND GPIO driver.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param addr 8 bit part of an address to write to the NAND flash device.
 */
static void nand_gpio_write_addr(const uint8_t addr)
{
	//_nand_gpio_write_io(nfd, addr);


	//gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_we);
	gpio_we_reset();

	_nand_gpio_write_io(addr);
	//_nand_gpio_write_io(addr);
	//_nand_gpio_write_io(addr);

	////_delay_ns(t_als); //porting - end
	//nDelay(t_als);

//	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_we);
	gpio_we_set();

	////_delay_ns(t_wh); //porting - end
	//nDelay(t_wh);

	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
}

/*! \brief Configure the NAND GPIO driver.
 *
 *  This function will configure the GPIO lines to the NAND flash device,
 *  and call the \ref nand_gpio_reset and \ref nand_gpio_read_id functions.
 *
 *  After this function has been called the user must set up the
 *  block_status array in the nand_driver_data struct and call the
 *  \ref nand_create_badblocks_table function.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *
 *  \return 0 on success, an error number elsewise.
 */
int32_t nand_gpio_init(struct nand_driver_data *nfd)
{
	//int32_t i;
	int32_t retval;


	nfd->bad_table.block_status = bad_block_table;


	/* Configure GPIO for NAND device */
	//gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_ce); //not used
	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
	gpio_ce_reset();

//	gpio_enable_pin_input(nfd->gpio_cont_ddr, nfd->gpio_rb); //not used
//	gpio_enable_pin_pull_up(nfd->gpio_cont_port, nfd->gpio_rb); //not used
//	gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_re); //not used
//	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_re);
	gpio_re_set();
//	gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_we);//not used
//	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_we);
	gpio_we_set();

//write protection is not used.
//	if (nfd->gpio_wp >= 0) {
//		gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_wp);
//		gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_wp);
//		//gpio_enable_pin_input(nfd->gpio_cont_ddr, nfd->gpio_wp);
//		//gpio_disable_pin_pull_up(nfd->gpio_cont_port, nfd->gpio_wp);
//	}

//	gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_ale); //not used
//	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ale);
	gpio_ale_set();

//	gpio_enable_pin_output(nfd->gpio_cont_ddr, nfd->gpio_cle); //not used
//	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_cle);
	gpio_cle_reset();

	/* Enable GPIO for NAND flash I/O port. */
	//gpio_enable_pin_output(nfd->gpio_io_ddr, 0xFF);
	//gpio_clr_gpio_pin(nfd->gpio_io_port, 0xFF);

	//for (i = 0; i < nfd->gpio_io_size; i++) {
		//gpio_enable_gpio_pin((32 * nfd->gpio_io_port) +
				//nfd->gpio_io_offset + i);
		//gpio_disable_pin_pull_up((32 * nfd->gpio_io_port) +
				//nfd->gpio_io_offset + i);
	//}

//	retval = nand_gpio_reset(nfd);
//	if (retval) {
//		return retval;
//	}

	retval = nand_gpio_read_id(nfd);
	if (retval) {
		return retval;
	}



	return 0;
}

/*! \brief Reset the NAND flash device.
 *
 *  This function resets the NAND flash device by sending the reset command.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *
 *  \return 0 on success, an error number elsewise.
 */
static int32_t nand_gpio_reset(struct nand_driver_data *nfd)
{
#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	if (nand_gpio_write_cmd(nfd, NAND_CMD_RESET, 0, 0)) {
		return -EBUSY;
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return 0;
}


/*! \brief Read ID from NAND flash.
 *
 *  This function reads the ID from the NAND flash device. These ID bytes are
 *  used to fill in information about the flash producer, model, page size,
 *  block size, plane size, etc.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *
 *  \return 0 on success, an error number elsewise.
 */
static int32_t nand_gpio_read_id(struct nand_driver_data *nfd)
{
	uint32_t  spare_size;
	uint8_t maker_code;
	uint8_t device_code;
	uint8_t chip_data;
	uint8_t size_data;
	uint8_t plane_data;
	uint8_t plane_size;


#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	nand_gpio_write_cmd(nfd, NAND_CMD_ID, 0, 0);

	/* Set GPIO I/O port in input mode. */
	//gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF);
	//gpio_disable_pin_pull_up(nfd->gpio_io_port, 0xFF);

	/* Set GPIO I/O port in input mode. */
//	gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF); //not used
//	gpio_enable_pin_pull_up(nfd->gpio_io_port, 0xFF); //not used

//	maker_code = nand_gpio_read_io(nfd);
//	device_code = nand_gpio_read_io(nfd);
//	chip_data = nand_gpio_read_io(nfd);
//	size_data = nand_gpio_read_io(nfd);
//	plane_data = nand_gpio_read_io(nfd);

//
//	gpio_cle_reset();
//	gpio_we_set();
//	gpio_ale_reset();
//	gpio_re_set();
//
//	nDelay(t_wp);
//
//
//	gpio_cle_set();
//
//	_nand_gpio_write_io(NAND_COMMAND_READID);
//
//	gpio_we_reset();
//
//
//
//	nDelay(t_wp);
//
//	gpio_cle_reset();
//
//	gpio_we_set();
//
//	gpio_ale_set();
//
//	nDelay(t_alh);
//
//	_nand_gpio_write_io(0);
//
//	nDelay(t_alh);
//
//	gpio_we_reset();
//
//	nDelay(t_alh);
//
//	gpio_we_set();
//
//	//gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_cle);
//
//	nDelay(t_alh);
//
//	gpio_ale_reset();
//
//	nDelay(t_alh);



	//2018/11/19/leeyongjun: No plane data..  -- 1D 00 F1 01, no plane data.
	maker_code = nand_gpio_read_io();
	device_code = nand_gpio_read_io();
	chip_data = nand_gpio_read_io();
	size_data = nand_gpio_read_io();

   // plane data 없음. 	
	plane_data = nand_gpio_read_io();


#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	/* Fill the NAND structure parameters */
	nfd->info.maker_code = maker_code;
	nfd->info.device_code = device_code;
	nfd->info.page_size  = 0x400 << (size_data & 0x03);  //0, 1bit 
	//nfd->info.page_shift = ctz(nfd->info.page_size); //porting. - end.
	nfd->info.page_shift = ctz32(nfd->info.page_size);
	// 1 block = 64 pages. 
	nfd->info.block_size = (64UL * 1024UL) << ((size_data & 0x30) >> 4); // 4,5bit

	nfd->info.num_planes = (1 << ((plane_data >> 2) & 0x03));

	plane_size = (plane_data >> 4) & 0x07;

	/* Store the plane size in bytes. */
	switch (plane_size) {
		case 0x0:
			nfd->info.plane_size = 1UL << 23;
			break;
		case 0x1:
			nfd->info.plane_size = 1UL << 24;
			break;
		case 0x2:
			nfd->info.plane_size = 1UL << 25;
			break;
		case 0x3:
			nfd->info.plane_size = 1UL << 26;
			break;
		case 0x4:
			nfd->info.plane_size = 1UL << 27;
			break;
		case 0x5:
			nfd->info.plane_size = 1UL << 28;
			break;
		case 0x6:
			nfd->info.plane_size = 1UL << 29;
			break;
		case 0x7:
			nfd->info.plane_size = 1UL << 30;
			break;
		default:
			return -EINVAL;
	}

	if ((size_data & 0x40) == 0) {
		nfd->info.bus_width = 8;
	} else {
		nfd->info.bus_width = 16;
	}

//전체 block number 의미 ??? 1024
	nfd->info.num_blocks = 1024; //nfd->info.num_planes * nfd->info.plane_size /nfd->info.block_size;
	nfd->info.pages_per_block = nfd->info.block_size / nfd->info.page_size;
	//nfd->info.block_shift = ctz(nfd->info.pages_per_block); //porting. - end
	nfd->info.block_shift = ctz32(nfd->info.pages_per_block);

	if (nfd->info.page_size >= 512) {
		nfd->info.badblock_offset = NAND_LARGE_BAD_BLOCK_POSITION;
	} else {
		nfd->info.badblock_offset = NAND_SMALL_BAD_BLOCK_POSITION;
	}

	spare_size = (8 << ((size_data & 0x04) >> 2)) *(nfd->info.page_size / 512);

	switch (spare_size) {
	case 16:
		nfd->info.oob = &nand_oob_16;
		break;
	case 64:
		nfd->info.oob = &nand_oob_64;
		break;
	default:
		return -EINVAL;
	}

	nfd->info.page_oob_size = ((nfd->info.page_size) + (nfd->info.oob->size));
	nfd->info.block_oob_size = (nfd->info.page_oob_size*nfd->info.pages_per_block);

	return 0;
}

/*! \brief Creates a bad blocks table.
 *
 *  This function creates a bad block table of the entire flash. It is vital
 *  that the nand_driver_data struct is proper set up before calling
 *  this function because it will assume the rather large block_status array
 *  is initialized.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *
 *  \return 0 on success, an error number elsewise.
 */


int32_t nand_gpio_create_badblocks_table(struct nand_driver_data *nfd)
{
	int32_t i;
	int32_t retval;
	uint64_t num_bad_blocks = 0;

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	for (i = 0; i < nfd->info.num_blocks; i++) {
		retval = nand_gpio_block_checkbad(nfd, i);
		if (retval != NAND_BLOCK_OK) {
			++num_bad_blocks;
			nand_gpio_block_markbad(nfd, i);
		}
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return num_bad_blocks;
}

/*! \brief Checks if a block in the bad blocks table is bad.
 *
 *  This function will check if a block in the bad blocks table is bad.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to check, starting at position 0.
 *
 *  \return 0 on ok block, an error number elsewise.
 */
static int32_t nand_gpio_block_isbad(struct nand_driver_data *nfd,
		const uint32_t block)
{
//	if(nfd->bad_table.block_status[block] == NAND_BLOCK_BAD) {
	if(bad_block_table[block] == NAND_BLOCK_BAD) {
		return -EIO;
	}

	return 0;
}

void nand_gpio_block_markbad(struct nand_driver_data *nfd,
		const uint32_t block)
{
}

/*! \brief Marks a block in the bad blocks table bad.
 *
 *  This function will set a block to be bad in the bad blocks table.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to mark bad, starting at position 0.
 */
void nand_gpio_block_markok(struct nand_driver_data *nfd,
		const uint32_t block)
{
//	nfd->bad_table.block_status[block] = NAND_BLOCK_OK;
	bad_block_table[block] = NAND_BLOCK_OK;
}

/*! \brief Checks if a block in NAND flash is bad.
 *
 *  This function will read the bad block byte in the NAND flash and check
 *  if the block is bad. This function is used by the
 *  \ref nand_gpio_create_badblocks_table.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to check, starting at position 0.
 *
 *  \return 0 on ok block, an error number elsewise.
 */
 // 사용 안함, 
static int32_t nand_gpio_block_checkbad(struct nand_driver_data *nfd,
		const uint32_t block)
{
	int32_t retval;
	uint8_t tmp_read;
	uint64_t offset = nfd->info.page_size + nfd->info.badblock_offset;
	uint64_t addr = block << nfd->info.block_shift;

	if (nfd->info.bus_width == 16) {
		offset &= 0xfe;
	}

	/*
	 * Read first page into NAND flash cache. Byte offset in page
	 * (2 bytes) and page + block address (3 bytes).
	 *
	 * Jump to the bad block information location in this block and
	 * read a byte, should be 0xff.
	 */
	retval = nand_gpio_write_cmd(nfd, NAND_CMD_READ, addr, offset);
	if (retval) {
		return retval;
	}

	/* Set GPIO I/O port in input mode. */
//	gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF); //not used
//	gpio_disable_pin_pull_up(nfd->gpio_io_port, 0xFF); //not used

//	tmp_read = nand_gpio_read_io(nfd);
	tmp_read = nand_gpio_read_io();

	if (!(tmp_read & 0xff)) {
		nand_gpio_block_markbad(nfd, block);
		return NAND_BLOCK_BAD;
	} else {
		nand_gpio_block_markok(nfd, block);
	}

	/* Check second page as well if first page seems to be OK. */
	if (!nand_gpio_block_isbad(nfd, block)) {
		retval = nand_gpio_write_cmd(nfd, NAND_CMD_READ,
				addr + 1, offset);
		if (retval) {
			return retval;
		}

		/* Set GPIO I/O port in input mode. */
//		gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF); //not used
//		gpio_disable_pin_pull_up(nfd->gpio_io_port, 0xFF); //not used

//		tmp_read = nand_gpio_read_io(nfd);
		tmp_read = nand_gpio_read_io();
		if (!(tmp_read & 0xff)) {
			nand_gpio_block_markbad(nfd, block);
			return NAND_BLOCK_BAD;
		} else {
			nand_gpio_block_markok(nfd, block);
		}
	}

	return 0;
}

#ifdef NAND_MARK_BLOCKS_BAD
/*! \brief Set block in NAND flash to be bad.
 *
 *  This function will set a block in the NAND flash to be bad, this is used
 *  to mark bad blocks permanently in the NAND flash. Use with care.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to set ok, starting at position 0.
 *
 *  \return 0 on success, an error number elsewise.
 */
static int32_t nand_gpio_block_setbad(struct nand_driver_data *nfd,
		const uint32_t block)
{
	int32_t retval;
	uint64_t addr = block << nfd->info.block_shift;
	uint64_t offset = nfd->info.page_size + nfd->info.badblock_offset;

	if (nfd->info.bus_width == 16) {
		offset &= 0xfe;
	}

	nand_gpio_block_markbad(nfd, block);

	retval = nand_gpio_erase(nfd, block);
	if (retval) {
		return retval;
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_PROGRAM, addr, offset);
	if (retval) {
		return retval;
	}

	//_delay_ns(t_adl);
	nDelay(t_adl);

	nand_gpio_write_io(nfd, 0);

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_PROGRAM_COMPLETE, 0, 0);
	if (retval) {
		return retval;
	}

	/* Check the write sequence operation. */
	if (nand_gpio_command_failed(nfd)) {
		return -EIO;
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return 0;
}

#ifdef NAND_MARK_BLOCKS_OK
/*! \brief Set block in NAND flash to be ok.
 *
 *  This function will set a block in the NAND flash to be ok, this is used
 *  if a block has been faulty marked as a bad block. Use with care.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to set ok, starting at position 0.
 *
 *  \return 0 on success, an error number elsewise.
 */
static int32_t nand_gpio_block_setok(struct nand_driver_data *nfd,
		const uint32_t block)
{
	int32_t retval = nand_gpio_erase(nfd, block);
	if (retval) {
		return retval;
	}

	return nand_gpio_block_checkbad(nfd, block);
}
#endif
#endif

#if NAND_ECC_TYPE == NAND_ECC_SW
/*! \brief Read ECC data from the spare area of the NAND flash.
 *
 *  This function reads ECC data from the spare area of the NAND flash.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param ecc Pointer to variable to put the read ECC data.
 *  \param block Block number for the related data, starting at position 0.
 *  \param offset Offset into the block for the related data.
 *
 *  \return Number of bytes read, an error number elsewise.
 */
int32_t nand_gpio_read_oob_ecc(struct nand_driver_data *nfd, uint64_t *ecc,
		const uint32_t block, const uint32_t offset)
{
	int32_t i;
	int32_t retval;
	int32_t ecc_offset;
	int32_t ecc_page_chunk;
	uint64_t col_addr;
	uint64_t row_addr;

	/* Known bad block? */
	if(nand_gpio_block_isbad(nfd, block)) {
		return -EIO;
	}

	/* Offset must be in ecc chunk size order. */
	if (offset % NAND_ECC_CHUNK_SIZE || block >= nfd->info.num_blocks
			|| offset >= nfd->info.block_size) {
		return -EINVAL;
	}

	/* Get position about where ECC for current chunk is. */
	ecc_page_chunk = (offset - ((offset / nfd->info.page_size)
			* nfd->info.page_size)) / NAND_ECC_CHUNK_SIZE;
	ecc_page_chunk *= NAND_ECC_BYTES_PER_CHUNK;
	ecc_offset = nfd->info.oob->eccpos[ecc_page_chunk];

	/* Initial col_addr must be page size + jump into ECC offset. */
	col_addr = nfd->info.page_size + ecc_offset;
	row_addr = (block << nfd->info.block_shift) +
			(offset >> nfd->info.page_shift);

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_READ, row_addr, col_addr);
	if (retval) {
		return retval;
	}

	*ecc = 0;

	/* Fill the Page Buffer with main area data. */
	for (i = 0; i < NAND_ECC_BYTES_PER_CHUNK; ) {
		/* Set GPIO I/O port in input mode. */
//		gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF); //not used
//		gpio_disable_pin_pull_up(nfd->gpio_io_port, 0xFF); //not used

		if (nfd->info.bus_width == 8) {
//			*ecc |= nand_gpio_read_io(nfd) << (i * 8);
			*ecc |= nand_gpio_read_io() << (i * 8);
			i++;
		} else {
//			*ecc |= nand_gpio_read_io(nfd) << (i * 16);
			*ecc |= nand_gpio_read_io() << (i * 16);
			i += 2;
		}

		/* Do we need to move the position in the read buffer? */
		if ((ecc_offset + (nfd->info.bus_width / 8))
				!= nfd->info.oob->eccpos[ecc_page_chunk
				               + (nfd->info.bus_width / 8)]) {
			/* Move read pointer. */
			col_addr = nfd->info.page_size
					+ nfd->info.oob->
						eccpos[ecc_page_chunk + i];
			retval = nand_gpio_write_cmd(nfd, NAND_CMD_RANDOM_READ,
					0, col_addr);
			if (retval) {
				return retval;
			}
		}

		ecc_page_chunk += nfd->info.bus_width / 8;
		ecc_offset = nfd->info.oob->eccpos[ecc_page_chunk];
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return nfd->info.oob->eccbytes;
}

/*! \brief Write ECC data into the spare area of the NAND flash.
 *
 *  This function writes ECC data into the spare area of the NAND flash.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param ecc ECC data to write into the spare area.
 *  \param block Block number for the related data, starting at position 0.
 *  \param offset Offset into the block for the related data.
 *
 *  \return Number of bytes written, an error number elsewise.
 */
int32_t nand_gpio_write_oob_ecc(struct nand_driver_data *nfd,
		const uint64_t ecc, const uint32_t block,
		const uint32_t offset)
{
	int32_t i;
	int32_t retval;
	int32_t ecc_offset;
	int32_t ecc_page_chunk;
	uint64_t col_addr;
	uint64_t row_addr;

	/* Known bad block? */
	if(nand_gpio_block_isbad(nfd, block)) {
		return -EIO;
	}

	/* Offset must be in ecc chunk size order. */
	if (offset % NAND_ECC_CHUNK_SIZE || block >= nfd->info.num_blocks
			|| offset >= nfd->info.block_size) {
		return -EINVAL;
	}

	/* Get position about where ECC for current chunk is. */
	ecc_page_chunk = (offset - ((offset / nfd->info.page_size)
			* nfd->info.page_size)) / NAND_ECC_CHUNK_SIZE;
	ecc_page_chunk *= NAND_ECC_BYTES_PER_CHUNK;
	ecc_offset = 1 + (offset/NAND_ECC_CHUNK_SIZE)*NAND_ECC_BYTES_PER_CHUNK;//nfd->info.oob->eccpos[ecc_page_chunk];

	/* Initial col_addr must be page size + jump into ECC offset. */
	col_addr = nfd->info.page_size + ecc_offset;
	row_addr = (block << nfd->info.block_shift) +
			(offset >> nfd->info.page_shift);

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_PROGRAM, row_addr, col_addr);
	if (retval) {
		return retval;
	}

	//_delay_ns(t_adl);
	//nDelay(t_adl);

	/* Fill the Page Buffer with main area data. */
	for (i = 0; i < NAND_ECC_BYTES_PER_CHUNK; ) {
		if (nfd->info.bus_width == 8) {
			//nand_gpio_write_io(nfd, (ecc >> (i * 8)) & 0xff);
			nand_gpio_write_io((ecc >> (i * 8)) & 0xff);
			i++;
		} else {
//			nand_gpio_write_io(nfd, (ecc >> (i * 16)) & 0xffff);
			nand_gpio_write_io((ecc >> (i * 16)) & 0xffff);
			i += 2;
		}

//		/* Do we need to move the position in the read buffer? */
//		if ((ecc_offset + (nfd->info.bus_width / 8))
//				!= nfd->info.oob->eccpos[ecc_page_chunk
//				               + (nfd->info.bus_width / 8)]) {
//			/* Move read pointer. */
//			col_addr = nfd->info.page_size + nfd->info.oob->
//					eccpos[ecc_page_chunk + i];
//			retval = nand_gpio_write_cmd(nfd, NAND_CMD_RANDOM_PROGRAM,
//					0, col_addr);
//			if (retval) {
//				return retval;
//			}
//		}

//		ecc_page_chunk += (nfd->info.bus_width / 8);
//		ecc_offset = nfd->info.oob->eccpos[ecc_page_chunk];
	}

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_PROGRAM_COMPLETE, 0, 0);
	if (retval) {
		return retval;
	}

	/* Check the write sequence operation. */
	if (nand_gpio_command_failed(nfd)) {
#ifdef NAND_MARK_BLOCKS_BAD
		nand_gpio_block_setbad(nfd, block);
#else
		nand_gpio_block_markbad(nfd, block);
#endif
		return -EIO;
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return nfd->info.oob->eccbytes;
}
#endif

/*! \brief Erase a block in the NAND flash.
 *
 *  This function erases the contents in a given block in the NAND flash.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to erase, starting at position 0.
 *
 *  \return 0 on success, an error number elsewise.
 */
int32_t nand_gpio_erase(struct nand_driver_data *nfd, const uint32_t block)
{
	int32_t retval;
	uint64_t addr;

	if (block >= nfd->info.num_blocks) {
		return -EINVAL;
	}

	/* Do NOT erase bad blocks. */
	if(nand_gpio_block_isbad(nfd, block)) {
		return -EIO;
	}

	addr = block << nfd->info.block_shift;

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_ERASE, addr, 0);

//	_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE1);
//
//	gpio_ale_set();
//	nand_gpio_write_addr((addr)       & 0xff);
//	nand_gpio_write_addr((addr >> 8)  & 0xff);
//	//nand_gpio_write_addr((addr >> 16) & 0xff);
//	gpio_ale_reset();
//	_nand_gpio_write_cmd(nfd, NAND_COMMAND_ERASE2);
//	retval = nand_gpio_wait_ready(nfd);

	if (retval) {
		return retval;
	}

	if (nand_gpio_command_failed(nfd)) {
#ifdef NAND_MARK_BLOCKS_BAD
		nand_gpio_block_setbad(nfd, block);
#else
		nand_gpio_block_markbad(nfd, block);
#endif
		return -EIO;
	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return 0;
}

/*! \brief Read data from the NAND flash.
 *
 *  This function reads data from the NAND flash at a given block and offset
 *  into a buffer.
 *
 *  \param nfd Pointer to the nand_driver_data struct which
 *             holds all vital data about the NAND GPIO driver.
 *  \param block Block number to read from, starting at position 0.
 *  \param offset Offset into the block to read from, starting at position 0.
 *  \param buf Buffer for storing the read data.
 *  \param count Number of bytes to read.
 *
 *  \return Number of bytes read, an error number elsewise.
 */
int32_t nand_gpio_read(struct nand_driver_data *nfd,
		const uint32_t block, const uint32_t offset,
		uint8_t *buf, const uint16_t count)
{
	int32_t i;
	int32_t retval;
	uint64_t col_addr;
	uint64_t row_addr;
	uint8_t *buf_char = buf;
	uint16_t *buf_short = (uint16_t *)buf;
	uint64_t end = offset - ((offset / nfd->info.page_size)
				* nfd->info.page_size) + count;

	/* Known bad block? */
	if(nand_gpio_block_isbad(nfd, block)) {
		return -EIO;
	}

//	if (end > nfd->info.page_size || block >= nfd->info.num_blocks
//			|| offset >= nfd->info.block_size) {
//		return -EINVAL;
//	}

	if (  (end > nfd->info.page_oob_size )|| block >= nfd->info.num_blocks
			|| offset >= nfd->info.block_size) {
		return -EINVAL;
	}

//	if (  (end > nfd->info.page_size + nfd->info.oob->size )|| block >= nfd->info.num_blocks
//			|| offset >= nfd->info.block_oob_size) {
//		return -EINVAL;
//	}

	col_addr = offset % nfd->info.page_size;
//	col_addr = offset % ((nfd->info.page_size) + (nfd->info.oob->size));
	row_addr = (block << nfd->info.block_shift) +
		(offset >> nfd->info.page_shift);

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_clr_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	//__disable_irq();

	retval = nand_gpio_write_cmd(nfd, NAND_CMD_READ, row_addr, col_addr);
	if (retval) {
		return retval;
	}

	//nDelay(0);
	/* Set GPIO I/O port in input mode. */
//	gpio_enable_pin_input(nfd->gpio_io_ddr, 0xFF); //not used
//	gpio_disable_pin_pull_up(nfd->gpio_io_port, 0xFF); //not used

	/* Fill the Page Buffer with main area data. */
	for (i = 0; i < count; ) {
		if (nfd->info.bus_width == 8) {
//			*buf_char = nand_gpio_read_io(nfd);
			*buf_char = nand_gpio_read_io();
			buf_char++;
			i++;
		} else {
//			*buf_short = nand_gpio_read_io(nfd);
			*buf_short = nand_gpio_read_io();
			buf_short++;
			i += 2;
		}
	}
	//__enable_irq();

//	for (i = 0; i < count; i += TX_BUFFER_SIZE) {
//		/* Check if the current endpoint can be written to and that the next sample should be stored */
//		while (!Endpoint_IsINReady()) USB_USBTask();
//
//		for (uint8_t tx = 0; tx < TX_BUFFER_SIZE; ++tx) {
//			Endpoint_Write_8(nand_gpio_read_io(nfd));
//		}
//
//		Endpoint_ClearIN();
//		//USB_USBTask();
//	}

#ifndef NAND_CE_ALWAYS_ACTIVE
	gpio_set_gpio_pin(nfd->gpio_cont_port, nfd->gpio_ce);
#endif

	return count;
}
