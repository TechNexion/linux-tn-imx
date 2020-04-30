# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the CAAM backend and dependent components
#
ifeq ($(CONFIG_CRYPTO_DEV_FSL_CAAM_DEBUG), y)
	ccflags-y := -DDEBUG
endif

obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM) += caam.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_JR) += caam_jr.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API) += caamalg.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI) += caamalg_qi.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_DESC) += caamalg_desc.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API) += caamhash.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API) += caamrng.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_PKC_API) += caam_pkc.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_SM) += sm_store.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_SM_TEST) += sm_test.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_SECVIO) += secvio.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_ECDSA_API) += caam_ecdsa.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_DESC) += caam_desc.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_UTIL) += caam_util.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_UTIL_BLACK_KEY) += black_keys.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_UTIL_BLOB) += blob.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_KEY_BLOB) += key_blob.o

caam-objs := ctrl.o
caam_jr-objs := jr.o key_gen.o error.o inst_rng.o
caam_pkc-y := caampkc.o pkc_desc.o
ifneq ($(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI),)
	ccflags-y += -DCONFIG_CAAM_QI
	caam-objs += qi.o
endif

obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_TK_API) += tag_object.o
