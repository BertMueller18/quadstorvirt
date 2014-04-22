/* 
 * Copyright (C) Shivaram Upadhyayula <shivaram.u@quadstor.com>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 2 as published by the Free Software Foundation
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, 
 * Boston, MA  02110-1301, USA.
 */

#include "cluster.h"
#include "gdevq.h"
#include "sense.h"
#include "tcache.h"
#include "vdevdefs.h"
#include "node_sock.h"
#include "node_ha.h"
#include "../common/cluster_common.h" 

void
node_msg_wait(struct node_msg *msg, struct node_sock *sock, int timo)
{
	int retval;

	msg->timestamp = ticks;
	retval = wait_for_done_timeout(msg->completion, timo);
	if (retval)
		return;
	debug_warn("msg timedout ticks %llu msg timestamp %llu cmd %d msg_id %llx xchg id %llx timo %d\n", (unsigned long long)ticks, (unsigned long long)msg->timestamp, msg->raw->msg_cmd, (unsigned long long)msg->raw->msg_id, (unsigned long long)msg->raw->xchg_id, timo);
	retval = node_cmd_hash_remove(sock->comm->node_hash, msg, msg->raw->msg_id);
	if (!retval) {
		wait_for_done(msg->completion);
	}
	else {
		node_sock_read_error(sock);
		debug_check(msg->resp);
	}
}

void
scsi_cmd_spec_fill(struct scsi_cmd_spec *spec, struct qsio_scsiio *ctio)
{
	spec->task_tag = ctio->task_tag;
	port_fill(spec->i_prt, ctio->i_prt);
	port_fill(spec->t_prt, ctio->t_prt);
	spec->r_prt = ctio->r_prt;
	spec->init_int = ctio->init_int;
	spec->task_attr = ctio->task_attr;
}

void
scsi_cmd_spec_generic_fill(struct scsi_cmd_spec_generic *spec, struct qsio_scsiio *ctio)
{
	memcpy(spec->cdb, ctio->cdb, sizeof(spec->cdb));
	spec->task_tag = ctio->task_tag;
	port_fill(spec->i_prt, ctio->i_prt);
	port_fill(spec->t_prt, ctio->t_prt);
	spec->r_prt = ctio->r_prt;
	spec->init_int = ctio->init_int;
	spec->task_attr = ctio->task_attr;
}

static int
node_resp_status(struct qsio_scsiio *ctio, struct node_msg *msg, struct node_comm *comm, struct node_sock *sock)
{
	struct node_msg *resp;
	struct raw_node_msg *raw;
	struct scsi_sense_spec *sense_spec;
	int need_sense, error_code;
	int sense_key = 0, asc = 0, ascq = 0;
	uint32_t info = 0;

	resp = msg->resp;

	if (unlikely(!resp))
		return -1;

	raw = resp->raw;
	if (raw->msg_status == NODE_STATUS_OK)
		return 0;

	if (!ctio)
		return raw->msg_status;
	need_sense = 0;
	error_code = SSD_CURRENT_ERROR;
	switch (raw->msg_status) {
	case NODE_STATUS_BUSY:
		ctio->scsi_status = SCSI_STATUS_BUSY;
		break;
	case NODE_STATUS_RESERV_CONFLICT:
		ctio->scsi_status = SCSI_STATUS_RESERV_CONFLICT;
		break;
	case NODE_STATUS_TARGET_NOT_FOUND:
		need_sense = 1;
		sense_key = SSD_KEY_ILLEGAL_REQUEST;
		asc = LOGICAL_UNIT_NOT_SUPPORTED_ASC;
		ascq = LOGICAL_UNIT_NOT_SUPPORTED_ASCQ;
		info = 0;
		break;
	case NODE_STATUS_INVALID_MSG:
	case NODE_STATUS_UNREGISTERED_NODE:
		need_sense = 1;
		sense_key = SSD_KEY_HARDWARE_ERROR;
		asc = INTERNAL_TARGET_FAILURE_ASC;
		ascq = INTERNAL_TARGET_FAILURE_ASCQ;
		info = 0;
		break;
	case NODE_STATUS_MEM_ALLOC_FAILURE:
		node_sock_read_error(sock);
		ctio->scsi_status = SCSI_STATUS_BUSY;
		break;
	case NODE_STATUS_SCSI_SENSE:
		need_sense = 1;
		sense_spec = (struct scsi_sense_spec *)(raw->data);
		debug_check(raw->dxfer_len != sizeof(*sense_spec));
		sense_key = sense_spec->sense_key;
		asc = sense_spec->asc;
		ascq = sense_spec->ascq; 
		info = sense_spec->info;
		error_code = sense_spec->error_code;
		if (msg->mirror)
			debug_warn("cmd sense sense key %x asc %x ascq %x error_code %x info %u\n", sense_key, asc, ascq, error_code, info);
		else
			debug_info("cmd sense sense key %x asc %x ascq %x error_code %x info %u\n", sense_key, asc, ascq, error_code, info);
		break;
	}

	if (need_sense) {
		if (sense_key == SSD_KEY_HARDWARE_ERROR)
			node_sock_read_error(sock);
		ctio_construct_sense(ctio, error_code, sense_key, info, asc, ascq);
	}
	return raw->msg_status;
}

int
node_cmd_remote_write_io(struct node_comm *comm, struct node_sock *sock, struct qsio_scsiio *ctio, struct node_msg *msg,  struct pgdata **pglist, int pglist_cnt, int timeout, int async)
{
	struct raw_node_msg *raw;
	struct pgdata_read_spec *source_spec;
	struct pgdata *pgtmp, *pgwrite;
	int i, retval;

	raw = msg->raw;
	raw->msg_cmd = NODE_MSG_WRITE_DATA;
	node_msg_init(msg);

	source_spec = pgdata_read_spec_ptr(raw);
	for (i = 0; i < pglist_cnt; i++, source_spec++) {
		pgtmp = pglist[i];
		if (!atomic_test_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags))
			continue;
		debug_check(!source_spec->amap_block);
		atomic_set_bit_short(PGDATA_NEED_REMOTE_IO, &source_spec->flags);
		if (pgtmp->comp_pgdata && lba_block_size(source_spec->amap_block) != LBA_SIZE) {
			pgwrite = pgtmp->comp_pgdata;
			debug_check(lba_block_size(source_spec->amap_block) !=  pgwrite->pg_len);
		}
		else {
			if (pgtmp->comp_pgdata) {
				pgdata_free(pgtmp->comp_pgdata);
				pgtmp->comp_pgdata = NULL;
			}
			pgwrite = pgtmp;
		}

		source_spec->csum = pgdata_csum(pgwrite, pgwrite->pg_len);
	}

	node_msg_compute_csum(raw);
	node_cmd_hash_insert(comm->node_hash, msg, raw->msg_id);
	node_sock_start(sock);
	retval = node_sock_write(sock, raw);
	if (unlikely(retval != 0)) {
		node_sock_end(sock);
		node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
		return retval;
	}

	for (i = 0; i < pglist_cnt; i++) {
		pgtmp = pglist[i];

		if (!atomic_test_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags))
			continue;

		if (pgtmp->comp_pgdata) {
			pgwrite = pgtmp->comp_pgdata;
		}
		else {
			pgwrite = pgtmp;
		}

		retval = node_sock_write_page(sock, pgwrite->page, pgwrite->pg_len);
		if (unlikely(retval != 0)) {
			node_sock_end(sock);
			node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
			return retval;
		}
	}
	node_sock_end(sock);

	if (async) {
		msg->async_wait = 1;
		return 0;
	}

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_resp_free(msg);
	return 0;
}

void
node_msg_copy_resp(struct node_msg *msg)
{
	struct node_msg *resp = msg->resp;

	free(msg->raw, M_NODE_RMSG);
	msg->raw = resp->raw;
	resp->raw = NULL;

	if (resp->pages) {
		msg->pages = resp->pages;
		msg->pg_count = resp->pg_count;
		resp->pages = NULL;
		resp->pg_count = 0;
	}

	node_resp_free(msg);
}

static int
node_cmd_remote_read_io(struct qsio_scsiio *ctio, struct node_comm *comm, struct node_sock *sock, struct node_msg *msg,  struct pgdata **pglist, int pglist_cnt, int timeout)
{
	struct raw_node_msg *raw;
	struct pgdata_read_spec *source_spec;
	struct pgdata *pgtmp;
	pagestruct_t **pages;
	struct pgdata_wlist read_list;
	int i, retval, pg_idx;

	raw = msg->raw;
	source_spec = pgdata_read_spec_ptr(raw);
	for (i = 0; i < pglist_cnt; i++, source_spec++) {
		pgtmp = pglist[i];

		if (!atomic_test_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags))
			continue;

		atomic_set_bit_short(PGDATA_NEED_REMOTE_IO, &source_spec->flags);
	}

	raw->msg_cmd = NODE_MSG_READ_DATA;
	node_msg_init(msg);
	retval = node_send_msg(sock, msg, raw->msg_id, 1);
	if (unlikely(retval != 0))
		return retval;

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	raw = msg->raw;

	source_spec = pgdata_read_spec_ptr(raw);
	pages = msg->pages;
	pg_idx = 0;

	STAILQ_INIT(&read_list);
	retval = 0;

	for (i = 0; i < pglist_cnt; i++, source_spec++) {
		pgtmp = pglist[i];

		if (!atomic_test_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags))
			continue;

		debug_check(pg_idx >= msg->pg_count);
		memcpy(pgdata_page_address(pgtmp), vm_pg_address(pages[pg_idx]), LBA_SIZE);
		if (unlikely(pgdata_csum(pgtmp, LBA_SIZE) != source_spec->csum)) {
			debug_warn("Invalid pgdata csum %x %x\n", pgdata_csum(pgtmp, LBA_SIZE), source_spec->csum);
			node_sock_read_error(sock);
			ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
			retval = -1;
			break;
		}
		pg_idx++;
	}

	if (msg->pages) {
		page_list_free(msg->pages, msg->pg_count);
		msg->pages = NULL;
		msg->pg_count = 0;
	}

	return retval;
}

int
node_cmd_read_io(struct tdisk *tdisk, struct qsio_scsiio *ctio, struct node_comm *comm, struct node_sock *sock, struct node_msg *msg, struct pgdata **pglist, int pglist_cnt, int remote, int timeout)
{
	struct raw_node_msg *raw;
	struct tcache *tcache;
	struct pgdata_read_spec *source_spec;
	struct pgdata *pgtmp;
	struct bdevint *bint, *prev_bint = NULL;
	struct pgdata_wlist read_list;
	int i, retval, need_remote_io = 0, need_uncomp = 0;
#ifdef ENABLE_STATS
	uint32_t start_ticks;
#endif

	source_spec = pgdata_read_spec_ptr(msg->raw);
	tcache = tcache_alloc(pglist_cnt);
	STAILQ_INIT(&read_list);

	for (i = 0; i < pglist_cnt; i++, source_spec++) {
		pgtmp = pglist[i];

		pgtmp->amap_block = source_spec->amap_block;
		pgtmp->flags = source_spec->flags;

		TDISK_INC(tdisk, read_total, 1);
		if (!source_spec->amap_block) {
			debug_check(!pgtmp->page);
			continue;
		}

		if (atomic_test_bit_short(PGDATA_FROM_RCACHE, &source_spec->flags)) {
			debug_check(!pgtmp->page);
			TDISK_INC(tdisk, from_rcache, 1);
			STAILQ_INSERT_TAIL(&read_list, pgtmp, t_list);
			continue;
		}

		if (pgdata_in_read_list(tdisk, pgtmp, &read_list, 0)) {
			TDISK_INC(tdisk, from_read_list, 1);
			continue;
		}

		debug_check(pgtmp->page);
		retval = pgdata_alloc_page(pgtmp, 0);
		if (unlikely(retval != 0)) {
			debug_warn("allocating for pgdata page failed\n");
			ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
			tcache_put(tcache);
			return -1;
		}

		if (remote) {
			need_remote_io++;
			TDISK_INC(tdisk, remote_reads, 1);
			atomic_set_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags);
			atomic_set_bit(PGDATA_SKIP_UNCOMP, &pgtmp->flags);
			continue;
		}

		if (!prev_bint || (prev_bint->bid != BLOCK_BID(pgtmp->amap_block))) {
			bint = bdev_find(BLOCK_BID(source_spec->amap_block));
			if (unlikely(!bint)) {
				need_remote_io++;
				TDISK_INC(tdisk, remote_reads, 1);
				atomic_set_bit(PGDATA_NEED_REMOTE_IO, &pgtmp->flags);
				atomic_set_bit(PGDATA_SKIP_UNCOMP, &pgtmp->flags);
				continue;
			}
			prev_bint = bint;
		}
		else {
			bint = prev_bint;
		}

		TDISK_INC(tdisk, local_reads, 1);
		if (lba_block_size(source_spec->amap_block) != LBA_SIZE)
			need_uncomp++;
		retval = tcache_add_page(tcache, pgtmp->page, BLOCK_BLOCKNR(source_spec->amap_block), bint, lba_block_size(source_spec->amap_block), QS_IO_READ);
		if (unlikely(retval != 0)) {
			debug_warn("tcache add page failed\n");
			ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
			tcache_put(tcache);
			return -1;
		}
	}

	if (atomic_read(&tcache->bio_remain))
		tcache_entry_rw(tcache, QS_IO_READ);
	else 
		wait_complete(tcache->completion);


	if (need_remote_io) {
		TDISK_TSTART(start_ticks);
		retval = node_cmd_remote_read_io(ctio, comm, sock, msg,  pglist, pglist_cnt, timeout);
		TDISK_TEND(tdisk, remote_read_io_ticks, start_ticks);
		if (unlikely(retval != 0)) {
			wait_for_done(tcache->completion);
			tcache_put(tcache);
			return retval;
		}
	}

	wait_for_done(tcache->completion);

	if (atomic_test_bit_short(TCACHE_IO_ERROR, &tcache->flags)) {
		tcache_put(tcache);
		debug_warn("tcache data error\n");
		ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
		return -1;
	}

	tcache_read_comp(tcache);

	tcache_put(tcache);

	raw = msg->raw;
	raw->msg_cmd = NODE_MSG_READ_DONE;
	node_msg_init(msg);
	raw->dxfer_len = 0;
	node_send_msg(sock, msg, 0, 0);

	if (!need_uncomp)
		return 0;

	retval = pgdata_post_read_io(pglist, pglist_cnt, NULL, 0, 0, 0);
	if (unlikely(retval != 0)) {
		debug_warn("pgdata post read io failed\n");
		ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
		return retval;
	}

	return 0;
}

int
node_cmd_write_done(struct qsio_scsiio *ctio, struct node_comm *comm, struct node_sock *sock, struct node_msg *msg, int timeout)
{
	int retval;

	msg->raw->msg_cmd = NODE_MSG_WRITE_DONE;
	node_msg_init(msg);
	msg->raw->dxfer_len = 0;

	retval = node_send_msg(sock, msg, msg->raw->msg_id, 1);
	if (unlikely(retval != 0))
		return retval;

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	return 0;

}

int
node_verify_setup(struct node_comm *comm, struct node_sock *sock, struct node_msg *msg, struct qsio_scsiio *ctio, int timeout, int async)
{
	struct raw_node_msg *raw;
	struct pgdata_read_spec *source_spec;
	struct pgdata *pgdata;
	struct pgdata **pglist;
	int i, retval;

	raw = msg->raw;
	pglist = (struct pgdata **)(ctio->data_ptr);
	source_spec = pgdata_read_spec_ptr(raw);

	for (i = 0; i < ctio->pglist_cnt; i++, source_spec++) {
		if (!atomic_test_bit_short(DDBLOCK_ENTRY_FOUND_DUPLICATE, &source_spec->flags))
			continue;

		pgdata = pglist[i];
		source_spec->csum = pgdata_csum(pgdata, LBA_SIZE);
	}

	raw->msg_cmd = NODE_MSG_VERIFY_DATA;
	node_msg_init(msg);
	node_msg_compute_csum(raw);
	node_cmd_hash_insert(comm->node_hash, msg, raw->msg_id);
	node_sock_start(sock);
	retval = node_sock_write(sock, raw);
	if (unlikely(retval != 0)) {
		node_sock_end(sock);
		node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
		return retval;
	}

	source_spec = pgdata_read_spec_ptr(raw);
	for (i = 0; i < ctio->pglist_cnt; i++, source_spec++) {
		if (!atomic_test_bit_short(DDBLOCK_ENTRY_FOUND_DUPLICATE, &source_spec->flags))
			continue;

		pgdata = pglist[i];
		retval = node_sock_write_page(sock, pgdata->page, pgdata->pg_len);
		if (unlikely(retval != 0)) {
			node_sock_end(sock);
			node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
			return retval;
		}
	}
	node_sock_end(sock);

	if (async) {
		msg->async_wait = 1;
		return 0;
	}

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	return 0;
}

int
node_comp_setup(struct node_comm *comm, struct node_sock *sock, struct node_msg *msg, struct qsio_scsiio *ctio, int timeout, int async)
{
	struct raw_node_msg *raw;
	struct pgdata_read_spec *source_spec, *tmp;
	struct pgdata *pgdata;
	struct pgdata **pglist;
	int i, retval;
	struct pgdata_wlist pending_list;
	uint64_t id;

	raw = msg->raw;
	source_spec = pgdata_read_spec_ptr(raw);
	pglist = (struct pgdata **)(ctio->data_ptr);

	STAILQ_INIT(&pending_list);
	for (i = 0; i < ctio->pglist_cnt; i++, source_spec++) {
		pgdata = pglist[i];
		pgdata->lba = i; /* id to use */

		if (atomic_test_bit_short(DDBLOCK_ZERO_BLOCK, &source_spec->flags))
			continue;

		if (atomic_test_bit_short(DDBLOCK_ENTRY_FOUND_DUPLICATE, &source_spec->flags))
			continue;

		if (pgdata->comp_pgdata)
			continue;

		gdevq_comp_insert(pgdata);
		STAILQ_INSERT_TAIL(&pending_list, pgdata, t_list);
	}

	source_spec = pgdata_read_spec_ptr(raw);
	while ((pgdata = STAILQ_FIRST(&pending_list)) != NULL) {
		STAILQ_REMOVE_HEAD(&pending_list, t_list);
		wait_for_done(pgdata->completion);
		if (pgdata->comp_pgdata) {
			id = pgdata->lba;
			tmp = &source_spec[id];
			debug_check(tmp->amap_block);
			SET_BLOCK_SIZE(tmp->amap_block, pgdata->comp_pgdata->pg_len);
		}
	}

	raw->msg_cmd = NODE_MSG_WRITE_COMP_DONE;
	node_msg_init(msg);
	retval = node_send_msg(sock, msg, raw->msg_id, 1);
	if (unlikely(retval != 0))
		return retval;

	if (async) {
		msg->async_wait = 1;
		return 0;
	}

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	return 0;
}

int 
node_write_setup(struct node_comm *comm, struct node_sock *sock, struct node_msg *msg, struct qsio_scsiio *ctio, uint64_t lba, uint32_t transfer_length, uint64_t amap_write_id, int unaligned, int timeout, int cmd)
{
	struct raw_node_msg *raw;
	struct pgdata_spec *source_spec;
	struct scsi_cmd_spec *cmd_spec;
	struct pgdata *pgdata;
	struct pgdata **pglist;
	int i, retval, done;

	raw = msg->raw;
	cmd_spec = scsi_cmd_spec_ptr(raw);
	scsi_cmd_spec_fill(cmd_spec, ctio);
	cmd_spec->transfer_length = transfer_length;
	cmd_spec->lba = lba;
	cmd_spec->pglist_cnt = ctio->pglist_cnt;
	cmd_spec->amap_write_id = amap_write_id;

	source_spec = pgdata_spec_ptr(raw);
	done = 0;
	pglist = (struct pgdata **)(ctio->data_ptr);
	for (i = 0; i < ctio->pglist_cnt; i++) {
		pgdata = pglist[i];
		wait_for_done(pgdata->completion);
		if (!unaligned && atomic_test_bit(DDBLOCK_ZERO_BLOCK, &pgdata->flags))
			continue;
		memcpy(source_spec->hash, pgdata->hash, sizeof(pgdata->hash));
		source_spec->flags = pgdata->flags;
		if (unaligned)
			source_spec->csum = pgdata_csum(pgdata, LBA_SIZE);
		else
			source_spec->csum = i;
		source_spec++;
		done++;
	}

	raw->msg_cmd = cmd;
	raw->dxfer_len = sizeof(struct scsi_cmd_spec) + (sizeof(struct pgdata_spec) * done);
	node_msg_compute_csum(raw);
	node_cmd_hash_insert(comm->node_hash, msg, raw->msg_id);
	node_sock_start(sock);
	retval = node_sock_write(sock, raw);
	if (unlikely(retval != 0)) {
		node_sock_end(sock);
		node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
		return retval;
	}

	if (unaligned) {
		for (i = 0; i < ctio->pglist_cnt; i++) {
			pgdata = pglist[i];
			retval = node_sock_write_page(sock, pgdata->page, pgdata->pg_len);
			if (unlikely(retval != 0)) {
				node_sock_end(sock);
				node_cmd_hash_remove(comm->node_hash, msg, raw->msg_id);
				return retval;
			}
		}
	}
	node_sock_end(sock);

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	return 0;
}

void
ctio_pglist_cleanup(struct qsio_scsiio *ctio)
{
	int i;
	struct pgdata **pglist, *pgdata;
	int norefs = ctio_norefs(ctio);

	pglist = (struct pgdata **)(ctio->data_ptr);
	for (i = 0; i < ctio->pglist_cnt; i++) {
		pgdata = pglist[i];
		if (pgdata->comp_pgdata) 
			pgdata_free(pgdata->comp_pgdata);

		if (!norefs)
			pgdata_free(pgdata);
		else
			pgdata_free_norefs(pgdata);
	}
	free(pglist, M_PGLIST);
	ctio->data_ptr = NULL;
	ctio->dxfer_len = 0;
	ctio->pglist_cnt = 0;
}

int 
node_read_setup(struct tdisk *tdisk, struct node_comm *comm, struct node_sock *sock, struct qsio_scsiio *ctio, struct node_msg *msg, uint64_t lba, struct pgdata **pglist, int pglist_cnt, uint32_t transfer_length, int timeout)
{
	struct raw_node_msg *raw;
	struct scsi_cmd_spec *cmd_spec;
	int retval;
	struct pgdata_read_spec *source_spec;
	struct pgdata_wlist read_list;
	int i, pg_idx;
	pagestruct_t **pages;
	struct pgdata *pgdata;
	int need_io, found;

	raw = msg->raw;
	raw->msg_cmd = NODE_MSG_READ_CMD;

	cmd_spec = scsi_cmd_spec_ptr(raw);
	scsi_cmd_spec_fill(cmd_spec, ctio);
	cmd_spec->lba = lba;
	cmd_spec->transfer_length = transfer_length;
	cmd_spec->pglist_cnt = pglist_cnt;

	retval = node_send_msg(sock, msg, raw->msg_id, 1);
	if (unlikely(retval != 0))
		return retval;

	node_msg_wait(msg, sock, timeout);
	retval = node_resp_status(ctio, msg, comm, sock);
	if (unlikely(retval != 0))
		return retval;

	node_msg_copy_resp(msg);
	raw = msg->raw;
	if (raw->mirror_status == NODE_STATUS_DO_LOCAL_READ)
		return 0;

	need_io = (node_cmd_status(msg) == NODE_CMD_NEED_IO);

	STAILQ_INIT(&read_list);
	debug_check(raw->dxfer_len != pgdata_read_spec_dxfer_len(pglist_cnt));
	source_spec = pgdata_read_spec_ptr(raw);
	pages = msg->pages;
	pg_idx = 0;
	retval = 0;
	for (i = 0; i < pglist_cnt; i++, source_spec++) {
		pgdata = pglist[i];
		pgdata->amap_block = source_spec->amap_block;
		pgdata->flags = source_spec->flags;

		if (!source_spec->amap_block) {
			pgdata_free_page(pgdata);
			pgdata_add_ref(pgdata, &pgzero);
			continue;
		}
		if (!atomic_test_bit_short(PGDATA_FROM_RCACHE, &source_spec->flags)) {
			if (need_io)
				continue;
			found = pgdata_in_read_list(tdisk, pgdata, &read_list, 0);
			debug_check(!found);
			continue;
		}
		debug_check(pg_idx >= msg->pg_count);
		vm_pg_ref(pages[pg_idx]);
		pgdata_free_page(pgdata);
		pgdata->page = pages[pg_idx];
		pg_idx++;
		if (pgdata_csum(pgdata, LBA_SIZE) != source_spec->csum) {
			debug_warn("Invalid pgdata csum\n");
			node_sock_read_error(sock);
			retval = -1;
			ctio_construct_sense(ctio, SSD_CURRENT_ERROR, SSD_KEY_HARDWARE_ERROR, 0, INTERNAL_TARGET_FAILURE_ASC, INTERNAL_TARGET_FAILURE_ASCQ);
			break;
		}
		STAILQ_INSERT_TAIL(&read_list, pgdata, t_list);
	}

	if (msg->pages) {
		page_list_free(msg->pages, msg->pg_count);
		msg->pages = NULL;
		msg->pg_count = 0;
	}

	raw->pg_count = 0;
	return retval;
}

static int
node_client_recv_pages(struct node_sock *sock, struct node_msg *resp, struct raw_node_msg *raw)
{
	pagestruct_t *page, **pages;
	int i, retval;

	pages = malloc((raw->pg_count * sizeof(pagestruct_t *)), M_PAGE_LIST, Q_WAITOK);
	for (i = 0; i < raw->pg_count; i++) {
		page = vm_pg_alloc(0);
		if (unlikely(!page)) {
			page_list_free(pages, i);
			node_sock_read_error(sock);
			return -1;
		}
		pages[i] = page;
		retval = node_sock_read_nofail(sock, vm_pg_address(page), LBA_SIZE);
		if (unlikely(retval != 0)) {
			page_list_free(pages, i+1);
			return -1;
		}
	}
	resp->pages = pages;
	resp->pg_count = raw->pg_count;
	return 0;
}

uint32_t recv_pages_ticks;

static int 
node_client_handle_resp(struct node_sock *sock, struct raw_node_msg *raw)
{
	struct node_msg *resp = NULL;
	int retval;
	struct node_msg *msg;
	uint16_t csum;
#ifdef ENABLE_STATS
	uint32_t start_ticks;
#endif

	resp = node_msg_alloc(raw->dxfer_len);
	memcpy(resp->raw, raw, sizeof(*raw));

	if (raw->dxfer_len) {
		retval = node_sock_read_nofail(sock, resp->raw->data, raw->dxfer_len);
		if (unlikely(retval != 0))
			goto err;

		csum = net_calc_csum16(resp->raw->data, resp->raw->dxfer_len);
		if (unlikely(csum != raw->data_csum)) {
			debug_warn("data csum mismatch\n");
			node_sock_read_error(sock);
			goto err;
		}
	}

	if (raw->pg_count) {
		GLOB_TSTART(start_ticks);
		retval = node_client_recv_pages(sock, resp, raw);
		GLOB_TEND(recv_pages_ticks, start_ticks);
		if (unlikely(retval != 0))
			goto err;
	}

	msg = node_cmd_lookup(sock->comm->node_hash, raw->msg_id, NULL, NULL);
	if (unlikely(!msg)) {
		debug_warn("Received response for unknown id %llx cmd %d \n", (unsigned long long)(raw->msg_id), raw->msg_cmd);
		node_msg_free(resp);
		return 0;
	}

	msg->resp = resp;
	wait_complete_all(msg->completion);
	return 0;
err:
	node_msg_free(resp);
	return -1;
}

int
node_client_recv(struct node_sock *sock)
{
	struct raw_node_msg raw;
	int retval;

	while (!sock_state_error(sock)) {
		retval = node_sock_read(sock, &raw, sizeof(raw));
		if (retval != 0)
			return -1;

		if (unlikely(!node_msg_csum_valid(&raw))) {
			debug_warn("Received msg with invalid csum\n");
			node_sock_read_error(sock);
			return -1;
		}

		retval = node_client_handle_resp(sock, &raw);
		if (unlikely(retval != 0))
			return retval;
	}
	return 0;
}
