/*
 * Copyright 2015 Red Hat Inc.
 * Copyright 2017 Edward O'Callaghan <funfunctor@folklore1984.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie
 *          Edward O'Callaghan
 */
#include <drm/drmP.h>
#include <drm/amdgpu_drm.h>
#include "amdgpu.h"

#define AUX_CH_BASE					0x5c00
/* extracted from: 'drivers/gpu/drm/radeon/nid.h' */
#define AUX_CONTROL					(AUX_CH_BASE + 0x00)
#define 	AUX_EN					(1 << 0)
#define 	AUX_LS_READ_EN				(1 << 8)
#define 	AUX_LS_UPDATE_DISABLE(x)		(((x) & 0x1) << 12)
#define 	AUX_HPD_DISCON(x)			(((x) & 0x1) << 16)
#define 	AUX_DET_EN				(1 << 18)
#define 	AUX_HPD_SEL(x)				(((x) & 0x7) << 20)
#define 	AUX_IMPCAL_REQ_EN			(1 << 24)
#define 	AUX_TEST_MODE				(1 << 28)
#define 	AUX_DEGLITCH_EN				(1 << 29)
#define AUX_SW_CONTROL					(AUX_CH_BASE + 0x01)
#define 	AUX_SW_GO				(1 << 0)
#define 	AUX_LS_READ_TRIG			(1 << 2)
#define 	AUX_SW_START_DELAY(x)			(((x) & 0xf) << 4)
#define 	AUX_SW_WR_BYTES(x)			(((x) & 0x1f) << 16)

#define AUX_SW_INTERRUPT_CONTROL			(AUX_CH_BASE + 0x03)
#define 	AUX_SW_DONE_INT				(1 << 0)
#define 	AUX_SW_DONE_ACK				(1 << 1)
#define 	AUX_SW_DONE_MASK			(1 << 2)
#define 	AUX_SW_LS_DONE_INT			(1 << 4)
#define 	AUX_SW_LS_DONE_MASK			(1 << 6)
#define AUX_SW_STATUS					(AUX_CH_BASE + 0x4)
#define 	AUX_SW_DONE				(1 << 0)
#define 	AUX_SW_REQ				(1 << 1)
#define 	AUX_SW_RX_TIMEOUT_STATE(x)		(((x) & 0x7) << 4)
#define 	AUX_SW_RX_TIMEOUT			(1 << 7)
#define 	AUX_SW_RX_OVERFLOW			(1 << 8)
#define 	AUX_SW_RX_HPD_DISCON			(1 << 9)
#define 	AUX_SW_RX_PARTIAL_BYTE			(1 << 10)
#define 	AUX_SW_NON_AUX_MODE			(1 << 11)
#define 	AUX_SW_RX_MIN_COUNT_VIOL		(1 << 12)
#define 	AUX_SW_RX_INVALID_STOP			(1 << 14)
#define 	AUX_SW_RX_SYNC_INVALID_L		(1 << 17)
#define 	AUX_SW_RX_SYNC_INVALID_H		(1 << 18)
#define 	AUX_SW_RX_INVALID_START			(1 << 19)
#define 	AUX_SW_RX_RECV_NO_DET			(1 << 20)
#define 	AUX_SW_RX_RECV_INVALID_H		(1 << 22)
#define 	AUX_SW_RX_RECV_INVALID_V		(1 << 23)

#define AUX_SW_DATA					(AUX_CH_BASE + 0x06)
#define AUX_SW_DATA_RW					(1 << 0)
#define AUX_SW_DATA_MASK(x)				(((x) & 0xff) << 8)
#define AUX_SW_DATA_INDEX(x)				(((x) & 0x1f) << 16)
#define AUX_SW_AUTOINCREMENT_DISABLE			(1 << 31)

/****************/

/*			    AUX_SW_RX_HPD_DISCON |	     \ */
#define AUX_RX_ERROR_FLAGS (AUX_SW_RX_OVERFLOW |	     \
			    AUX_SW_RX_PARTIAL_BYTE |	     \
			    AUX_SW_NON_AUX_MODE |	     \
			    AUX_SW_RX_SYNC_INVALID_L |	     \
			    AUX_SW_RX_SYNC_INVALID_H |	     \
			    AUX_SW_RX_INVALID_START |	     \
			    AUX_SW_RX_RECV_NO_DET |	     \
			    AUX_SW_RX_RECV_INVALID_H |	     \
			    AUX_SW_RX_RECV_INVALID_V)

#define AUX_SW_REPLY_GET_BYTE_COUNT(x) (((x) >> 24) & 0x1f)

#define BARE_ADDRESS_SIZE 3

static const uint32_t aux_offset[] =
{
	0x00, 0x1c, 0x38, 0x54, 0x70, 0x8c,
};

static void aux_fifo_put_first(struct amdgpu_device *adev, uint32_t block, uint8_t val)
{
	uint32_t reg32 = AUX_SW_DATA_MASK(val) | AUX_SW_AUTOINCREMENT_DISABLE;
	amdgpu_mm_wreg(adev, AUX_SW_DATA + block, reg32, false);
}

static void aux_fifo_put(struct amdgpu_device *adev, uint32_t block, uint8_t val)
{
	uint32_t reg32 = AUX_SW_DATA_MASK(val);
	amdgpu_mm_wreg(adev, AUX_SW_DATA + block, reg32, false);
}

static uint8_t aux_fifo_get(struct amdgpu_device *adev, uint32_t block)
{
	uint32_t reg32 = amdgpu_mm_rreg(adev, AUX_SW_DATA + block, false);
	return reg32 >> 8;
}

static int aux_do_transfer(struct amdgpu_device *adev, uint32_t block, size_t bytes)
{
	uint32_t status;
	int retry_count = 0;

	/* clear the ACK */
	amdgpu_mm_wreg(adev, AUX_SW_INTERRUPT_CONTROL + block, AUX_SW_DONE_ACK, false);

	/* write the size and GO bits */
	amdgpu_mm_wreg(adev, AUX_SW_CONTROL + block,
		       AUX_SW_WR_BYTES(bytes) | AUX_SW_GO, false);

	/* poll the status registers - TODO irq support */
	do {
		status = amdgpu_mm_rreg(adev, AUX_SW_STATUS + block, false);
		if (status & AUX_SW_DONE)
			break;

		usleep_range(100, 200);
	} while (retry_count++ < 1000);

	if (retry_count >= 1000) {
		DRM_ERROR("auxch hw never signalled completion, flags %08x\n",
			  status);
		return -EIO;
	}

	if (status & AUX_SW_RX_TIMEOUT)
		return -ETIMEDOUT;

	if (status & AUX_RX_ERROR_FLAGS) {
		DRM_DEBUG_KMS("dp_aux_ch flags not zero: %08x\n", status);
		return -EIO;
	}

	return 0;
}

ssize_t
amdgpu_native_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct amdgpu_i2c_chan *chan =
		container_of(aux, struct amdgpu_i2c_chan, aux);
	struct drm_device *dev = chan->dev;
	struct amdgpu_device *adev = dev->dev_private;
	int ret = 0, i;
	uint32_t tmp, block, ack = 0;
	int instance = chan->rec.i2c_id & 0xf;
	u8 *buf = msg->buffer;
	int bytes, msize;
	bool is_write = false;

	if (WARN_ON(msg->size > 16))
		return -E2BIG;

	if (WARN_ON(instance > 5))
		return -EINVAL;

	block = instance * 0x1c;

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
		is_write = true;
		break;
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		break;
	default:
		return -EINVAL;
	}

	/* work out two sizes required */
	msize = 0;
	bytes = BARE_ADDRESS_SIZE;
	if (msg->size) {
		msize = msg->size - 1;
		bytes++;
		if (is_write)
			bytes += msg->size;
	}

	mutex_lock(&chan->mutex);

	/* switch the pad to aux mode */
	tmp = amdgpu_mm_rreg(adev, chan->rec.mask_clk_reg, false);
	tmp |= (1 << 16);
	amdgpu_mm_wreg(adev, chan->rec.mask_clk_reg, tmp, false);

	/* setup AUX control register with correct HPD pin */
	tmp = amdgpu_mm_rreg(adev, AUX_CONTROL + block, false);

	tmp &= AUX_HPD_SEL(0x7);
	tmp |= AUX_HPD_SEL(chan->rec.hpd);
	tmp |= AUX_EN | AUX_LS_READ_EN;

	amdgpu_mm_wreg(adev, AUX_CONTROL + block, tmp, false);

	/* atombios appears to write this twice lets copy it */
	amdgpu_mm_wreg(adev, AUX_SW_CONTROL + block,
	       AUX_SW_WR_BYTES(bytes), false);
	amdgpu_mm_wreg(adev, AUX_SW_CONTROL + block,
	       AUX_SW_WR_BYTES(bytes), false);

	/* write the data header into the registers */
	/* request, address, msg size */
	aux_fifo_put_first(adev, block,
			   (msg->request << 4) | ((msg->address >> 16) & 0xf));
	aux_fifo_put(adev, block, msg->address >> 8);
	aux_fifo_put(adev, block, msg->address >> 0);
	aux_fifo_put(adev, block, msize);

	/* if we are writing - write the msg buffer */
	if (is_write) {
		for (i = 0; i < msg->size; i++)
			aux_fifo_put(adev, block, buf[i]);
	}

	ret = aux_do_transfer(adev, block, bytes);
	if (ret < 0)
		goto done;

	bytes = AUX_SW_REPLY_GET_BYTE_COUNT(tmp);
	if (bytes) {
		amdgpu_mm_wreg(adev, AUX_SW_DATA + block,
		       AUX_SW_DATA_RW | AUX_SW_AUTOINCREMENT_DISABLE, false);

		ack = aux_fifo_get(adev, block);

		if (buf) {
			for (i = 0; i < bytes - 1; i++)
				buf[i] = aux_fifo_get(adev, block);

			ret = bytes - 1;
		}
	}

	amdgpu_mm_wreg(adev, AUX_SW_INTERRUPT_CONTROL + block, AUX_SW_DONE_ACK, false);

	if (is_write)
		ret = msg->size;
done:
	mutex_unlock(&chan->mutex);

	if (ret >= 0)
		msg->reply = ack >> 4;
	return ret;
}
