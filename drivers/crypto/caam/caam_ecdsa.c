// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2019 NXP
 *
 * caam_ecdsa.c: A kernel module which exports functions to perform
 * ECDSA operations, (Sign, Verify, Key pair generation)
 * using CAAM's black key mechanism.
 *
 * It was mainly developed as PoC.
 * The driver can be used with any other application for demo purpose.
 *
 * This implementation uses CAAM built-in ECC domains.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "desc_constr.h"
#include "jr.h"
#include "desc.h"
#include "error.h"

/* Returns 1 for a valid signature */
#define ECDSA_VERIFY_SUCCESS	1
/* Returns 0 for an invalid signature */
#define ECDSA_VERIFY_FAIL		0

/* If the signature is incorrect, 0x86 status code is returned */
#define ECDSA_INVA_SIG_STATUS	0x86

/* ECDSA Protocol Data Block */
#define CAAM_PROTINFO_SEC_KEY	(0x01 << 2)
/* ECB-encrypted key
 * The bit is ignored for signature verification because only public
 * keys are used.
 */
#define CAAM_PROTOP_CTYPE		(0x10u << 27)
/**
 * When the PD (Predefined Domain) bit in the PDB is 1,
 * the ECDSEL (Elliptic Curve Domain Selection) field is
 * used  to select one of the built-in ECC domains
 */
#define CAAM_ECDSA_PD			(0x1 <<  22)

/* CAAM EC key pair generation*/
#define CAAM_CMD_ECC_GEN_KP		(0x2 << 24)

/* ECC prime field (Fp) */
#define ECC_DOMAIN_FP			0
/* ECC binary field (F2M) */
#define ECC_DOMAIN_F2M			1

/* ECC curves list */
#define CAAM_EC_CURVE_P_192 0
#define CAAM_EC_CURVE_P_224 1
#define CAAM_EC_CURVE_P_256 2
#define CAAM_EC_CURVE_P_384 3
#define CAAM_EC_CURVE_P_521 4
#define CAAM_EC_CURVE_PRIME192V1 5
#define CAAM_EC_CURVE_PRIME192V2 6
#define CAAM_EC_CURVE_PRIME192V3 7
#define CAAM_EC_CURVE_PRIME239V1 8
#define CAAM_EC_CURVE_PRIME239V2 9
#define CAAM_EC_CURVE_PRIME239V3 10
#define CAAM_EC_CURVE_PRIME256V1 11
#define CAAM_EC_CURVE_SECP112R1 12
#define CAAM_EC_CURVE_SECP160K1 13
#define CAAM_EC_CURVE_SECP160R1 14
#define CAAM_EC_CURVE_SECP160R2 15
#define CAAM_EC_CURVE_SECP192K1 16
#define CAAM_EC_CURVE_SECP192R1 17
#define CAAM_EC_CURVE_SECP224R1 18
#define CAAM_EC_CURVE_SECP224K1 19
#define CAAM_EC_CURVE_SECP256K1 20
#define CAAM_EC_CURVE_SECP256R1 21
#define CAAM_EC_CURVE_SECP384R1 22
#define CAAM_EC_CURVE_SECP521R1 23
#define CAAM_EC_CURVE_SECT113R1 24
#define CAAM_EC_CURVE_SECT163R1 25
#define CAAM_EC_CURVE_SECT163R2 26
#define CAAM_EC_CURVE_SECT163K1 27
#define CAAM_EC_CURVE_SECT193R1 28
#define CAAM_EC_CURVE_SECT193R2 29
#define CAAM_EC_CURVE_SECT233K1 30
#define CAAM_EC_CURVE_SECT233R1 31
#define CAAM_EC_CURVE_SECT239K1 32
#define CAAM_EC_CURVE_SECT283K1 33
#define CAAM_EC_CURVE_SECT283R1 34
#define CAAM_EC_CURVE_SECT409K1 35
#define CAAM_EC_CURVE_SECT409R1 36
#define CAAM_EC_CURVE_SECT571K1 37
#define CAAM_EC_CURVE_SECT571R1 38
#define CAAM_EC_CURVE_ANSIX9P192R1 39
#define CAAM_EC_CURVE_ANSIX9P224R1 40
#define CAAM_EC_CURVE_ANSIX9P256R1 41
#define CAAM_EC_CURVE_ANSIX9P384R1 42
#define CAAM_EC_CURVE_ANSIX9P521R1 43
#define CAAM_EC_CURVE_ANSIX9P160K1 44
#define CAAM_EC_CURVE_ANSIX9P160R2 45
#define CAAM_EC_CURVE_ANSIX9P224K1 46
#define CAAM_EC_CURVE_ANSIX9P160R1 47
#define CAAM_EC_CURVE_ANSIX9P192K1 48
#define CAAM_EC_CURVE_ANSIX9P256K1 49
#define CAAM_EC_CURVE_ANSIX9T163R2 50
#define CAAM_EC_CURVE_ANSIX9T233R1 51
#define CAAM_EC_CURVE_ANSIX9T283R1 52
#define CAAM_EC_CURVE_ANSIX9T163K1 53
#define CAAM_EC_CURVE_ANSIX9T233K1 54
#define CAAM_EC_CURVE_ANSIX9T283K1 55
#define CAAM_EC_CURVE_ANSIX9T571K1 56
#define CAAM_EC_CURVE_ANSIX9T163R1 57
#define CAAM_EC_CURVE_ANSIX9T193R1 58
#define CAAM_EC_CURVE_ANSIX9T193R2 59
#define CAAM_EC_CURVE_ANSIX9T239K1 60
#define CAAM_EC_CURVE_ANSIX9T409R1 61
#define CAAM_EC_CURVE_ANSIX9T571R1 62
#define CAAM_EC_CURVE_ANSIX9T409K1 63
#define CAAM_EC_CURVE_WTLS1 64
#define CAAM_EC_CURVE_WTLS3 65
#define CAAM_EC_CURVE_WTLS4 66
#define CAAM_EC_CURVE_WTLS5 67
#define CAAM_EC_CURVE_WTLS6 68
#define CAAM_EC_CURVE_WTLS7 69
#define CAAM_EC_CURVE_WTLS8 70
#define CAAM_EC_CURVE_WTLS9 71
#define CAAM_EC_CURVE_WTLS10 72
#define CAAM_EC_CURVE_WTLS11 73
#define CAAM_EC_CURVE_WTLS12 74
#define CAAM_EC_CURVE_ECDSA_256 75
#define CAAM_EC_CURVE_ECDSA_384 76
#define CAAM_EC_CURVE_ECDSA_521 77
#define CAAM_EC_CURVE_BRAINPOOLP160R1 78
#define CAAM_EC_CURVE_BRAINPOOLP160T1 79
#define CAAM_EC_CURVE_BRAINPOOLP192R1 80
#define CAAM_EC_CURVE_BRAINPOOLP192T1 81
#define CAAM_EC_CURVE_BRAINPOOLP224R1 82
#define CAAM_EC_CURVE_BRAINPOOLP224T1 83
#define CAAM_EC_CURVE_BRAINPOOLP256R1 84
#define CAAM_EC_CURVE_BRAINPOOLP256T1 85
#define CAAM_EC_CURVE_BRAINPOOLP320R1 86
#define CAAM_EC_CURVE_BRAINPOOLP320T1 87
#define CAAM_EC_CURVE_BRAINPOOLP384R1 88
#define CAAM_EC_CURVE_BRAINPOOLP384T1 89
#define CAAM_EC_CURVE_BRAINPOOLP512R1 90
#define CAAM_EC_CURVE_BRAINPOOLP512T1 91
#define CAAM_EC_CURVE_B_163 92
#define CAAM_EC_CURVE_B_233 93
#define CAAM_EC_CURVE_B_283 94
#define CAAM_EC_CURVE_B_409 95
#define CAAM_EC_CURVE_B_571 96
#define CAAM_EC_CURVE_K_163 97
#define CAAM_EC_CURVE_K_233 98
#define CAAM_EC_CURVE_K_283 99
#define CAAM_EC_CURVE_K_409 100
#define CAAM_EC_CURVE_K_571 101
#define CAAM_EC_CURVE_ECP_GROUP_19 102
#define CAAM_EC_CURVE_ECP_GROUP_20 103
#define CAAM_EC_CURVE_ECP_GROUP_21 104
#define CAAM_EC_CURVE_EC2N_GROUP_3 105
#define CAAM_EC_CURVE_EC2N_GROUP_4 106
#define CAAM_EC_CURVE_C2PNB163V1 107
#define CAAM_EC_CURVE_C2PNB163V2 108
#define CAAM_EC_CURVE_C2PNB163V3 109
#define CAAM_EC_CURVE_ECPRGF192RANDOM 110
#define CAAM_EC_CURVE_ECPRGF224RANDOM 111
#define CAAM_EC_CURVE_ECPRGF256RANDOM 112
#define CAAM_EC_CURVE_ECPRGF384RANDOM 113
#define CAAM_EC_CURVE_ECPRGF521RANDOM 114
#define CAAM_EC_CURVE_EC2NGF163RANDOM 115
#define CAAM_EC_CURVE_EC2NGF233RANDOM 116
#define CAAM_EC_CURVE_EC2NGF283RANDOM 117
#define CAAM_EC_CURVE_EC2NGF409RANDOM 118
#define CAAM_EC_CURVE_EC2NGF571RANDOM 119
#define CAAM_EC_CURVE_EC2NGF163KOBLITZ 120
#define CAAM_EC_CURVE_EC2NGF233KOBLITZ 121
#define CAAM_EC_CURVE_EC2NGF283KOBLITZ 122
#define CAAM_EC_CURVE_EC2NGF409KOBLITZ 123
#define CAAM_EC_CURVE_EC2NGF571KOBLITZ 124
#define CAAM_EC_CURVE_OAKLEY_3 125
#define CAAM_EC_CURVE_OAKLEY_4 126
#define CAAM_EC_CURVE_UNDEFINED	127

#define DEBUG

typedef enum { KEY_COLOR_NONE, KEY_COLOR_BLACK } key_color_t;

typedef struct{
	u8 caam_ec_id;
	u8 field_type;	/* ECC binary field (F2M) or prime field (Fp) */
	u32 l_len;		/* Size of the field (L) */
	u32 n_len;	/* Size of the subgroup (N) */
} ec_curve_data_t;

typedef struct{
	u32 cid;
	ec_curve_data_t data;
} ec_curve_t;


/* CAAM built-in curves with data */
static const ec_curve_t caam_ec_curve_list[] = {
	{CAAM_EC_CURVE_P_192, {0X00, 0, 24, 24} },
	{CAAM_EC_CURVE_P_224, {0X01, 0, 28, 28} },
	{CAAM_EC_CURVE_P_256, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_P_384, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_P_521, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_PRIME192V1, {0X00, 0, 24, 24} },
	{CAAM_EC_CURVE_PRIME192V2, {0X13, 0, 24, 24} },
	{CAAM_EC_CURVE_PRIME192V3, {0X14, 0, 24, 24} },
	{CAAM_EC_CURVE_PRIME239V1, {0X15, 0, 30, 30} },
	{CAAM_EC_CURVE_PRIME239V2, {0X16, 0, 30, 30} },
	{CAAM_EC_CURVE_PRIME239V3, {0X17, 0, 30, 30} },
	{CAAM_EC_CURVE_PRIME256V1, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_SECP112R1, {0X18, 0, 14, 14} },
	{CAAM_EC_CURVE_SECP160K1, {0X1B, 0, 20, 21} },
	{CAAM_EC_CURVE_SECP160R1, {0X1C, 0, 20, 21} },
	{CAAM_EC_CURVE_SECP160R2, {0X1D, 0, 20, 21} },
	{CAAM_EC_CURVE_SECP192K1, {0X1E, 0, 24, 24} },
	{CAAM_EC_CURVE_SECP192R1, {0X00, 0, 24, 24} },
	{CAAM_EC_CURVE_SECP224R1, {0X01, 0, 28, 28} },
	{CAAM_EC_CURVE_SECP224K1, {0X1F, 0, 28, 29} },
	{CAAM_EC_CURVE_SECP256K1, {0X20, 0, 32, 32} },
	{CAAM_EC_CURVE_SECP256R1, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_SECP384R1, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_SECP521R1, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_SECT113R1, {0X4B, 1, 15, 15} },
	{CAAM_EC_CURVE_SECT163R1, {0X4F, 1, 21, 21} },
	{CAAM_EC_CURVE_SECT163R2, {0X40, 1, 21, 21} },
	{CAAM_EC_CURVE_SECT163K1, {0X45, 1, 21, 21} },
	{CAAM_EC_CURVE_SECT193R1, {0X50, 1, 25, 25} },
	{CAAM_EC_CURVE_SECT193R2, {0X51, 1, 25, 25} },
	{CAAM_EC_CURVE_SECT233K1, {0X46, 1, 30, 29} },
	{CAAM_EC_CURVE_SECT233R1, {0X41, 1, 30, 30} },
	{CAAM_EC_CURVE_SECT239K1, {0X52, 1, 30, 30} },
	{CAAM_EC_CURVE_SECT283K1, {0X47, 1, 36, 36} },
	{CAAM_EC_CURVE_SECT283R1, {0X42, 1, 36, 36} },
	{CAAM_EC_CURVE_SECT409K1, {0X48, 1, 52, 51} },
	{CAAM_EC_CURVE_SECT409R1, {0X43, 1, 52, 52} },
	{CAAM_EC_CURVE_SECT571K1, {0X49, 1, 72, 72} },
	{CAAM_EC_CURVE_SECT571R1, {0X44, 1, 72, 72} },
	{CAAM_EC_CURVE_ANSIX9P192R1, {0X00, 0, 24, 24} },
	{CAAM_EC_CURVE_ANSIX9P224R1, {0X01, 0, 28, 28} },
	{CAAM_EC_CURVE_ANSIX9P256R1, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_ANSIX9P384R1, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_ANSIX9P521R1, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_ANSIX9P160K1, {0X1B, 0, 20, 21} },
	{CAAM_EC_CURVE_ANSIX9P160R2, {0X1D, 0, 20, 21} },
	{CAAM_EC_CURVE_ANSIX9P224K1, {0X1F, 0, 28, 29} },
	{CAAM_EC_CURVE_ANSIX9P160R1, {0X1C, 0, 20, 21} },
	{CAAM_EC_CURVE_ANSIX9P192K1, {0X1E, 0, 24, 24} },
	{CAAM_EC_CURVE_ANSIX9P256K1, {0X20, 0, 32, 32} },
	{CAAM_EC_CURVE_ANSIX9T163R2, {0X40, 1, 21, 21} },
	{CAAM_EC_CURVE_ANSIX9T233R1, {0X41, 1, 30, 30} },
	{CAAM_EC_CURVE_ANSIX9T283R1, {0X42, 1, 36, 36} },
	{CAAM_EC_CURVE_ANSIX9T163K1, {0X45, 1, 21, 21} },
	{CAAM_EC_CURVE_ANSIX9T233K1, {0X46, 1, 30, 29} },
	{CAAM_EC_CURVE_ANSIX9T283K1, {0X47, 1, 36, 36} },
	{CAAM_EC_CURVE_ANSIX9T571K1, {0X49, 1, 72, 72} },
	{CAAM_EC_CURVE_ANSIX9T163R1, {0X4F, 1, 21, 21} },
	{CAAM_EC_CURVE_ANSIX9T193R1, {0X50, 1, 25, 25} },
	{CAAM_EC_CURVE_ANSIX9T193R2, {0X51, 1, 25, 25} },
	{CAAM_EC_CURVE_ANSIX9T239K1, {0X52, 1, 30, 30} },
	{CAAM_EC_CURVE_ANSIX9T409R1, {0X43, 1, 52, 52} },
	{CAAM_EC_CURVE_ANSIX9T571R1, {0X44, 1, 72, 72} },
	{CAAM_EC_CURVE_ANSIX9T409K1, {0X48, 1, 52, 51} },
	{CAAM_EC_CURVE_WTLS1, {0X4A, 1, 15, 14} },
	{CAAM_EC_CURVE_WTLS3, {0X45, 1, 21, 21} },
	{CAAM_EC_CURVE_WTLS4, {0X4B, 1, 15, 15} },
	{CAAM_EC_CURVE_WTLS5, {0X4C, 1, 21, 21} },
	{CAAM_EC_CURVE_WTLS6, {0X18, 0, 14, 14} },
	{CAAM_EC_CURVE_WTLS7, {0X1C, 0, 20, 21} },
	{CAAM_EC_CURVE_WTLS8, {0X19, 0, 14, 15} },
	{CAAM_EC_CURVE_WTLS9, {0X1A, 0, 20, 21} },
	{CAAM_EC_CURVE_WTLS10, {0X46, 1, 30, 29} },
	{CAAM_EC_CURVE_WTLS11, {0X41, 1, 30, 30} },
	{CAAM_EC_CURVE_WTLS12, {0X01, 0, 28, 28} },
	{CAAM_EC_CURVE_ECDSA_256, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_ECDSA_384, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_ECDSA_521, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_BRAINPOOLP160R1, {0X05, 0, 20, 20} },
	{CAAM_EC_CURVE_BRAINPOOLP160T1, {0X06, 0, 20, 20} },
	{CAAM_EC_CURVE_BRAINPOOLP192R1, {0X07, 0, 24, 24} },
	{CAAM_EC_CURVE_BRAINPOOLP192T1, {0X08, 0, 24, 24} },
	{CAAM_EC_CURVE_BRAINPOOLP224R1, {0X09, 0, 28, 28} },
	{CAAM_EC_CURVE_BRAINPOOLP224T1, {0X0A, 0, 28, 28} },
	{CAAM_EC_CURVE_BRAINPOOLP256R1, {0X0B, 0, 32, 32} },
	{CAAM_EC_CURVE_BRAINPOOLP256T1, {0X0C, 0, 32, 32} },
	{CAAM_EC_CURVE_BRAINPOOLP320R1, {0X0D, 0, 40, 40} },
	{CAAM_EC_CURVE_BRAINPOOLP320T1, {0X0E, 0, 40, 40} },
	{CAAM_EC_CURVE_BRAINPOOLP384R1, {0X0F, 0, 48, 48} },
	{CAAM_EC_CURVE_BRAINPOOLP384T1, {0X10, 0, 48, 48} },
	{CAAM_EC_CURVE_BRAINPOOLP512R1, {0X11, 0, 64, 64} },
	{CAAM_EC_CURVE_BRAINPOOLP512T1, {0X12, 0, 64, 64} },
	{CAAM_EC_CURVE_B_163, {0X40, 1, 21, 21} },
	{CAAM_EC_CURVE_B_233, {0X41, 1, 30, 30} },
	{CAAM_EC_CURVE_B_283, {0X42, 1, 36, 36} },
	{CAAM_EC_CURVE_B_409, {0X43, 1, 52, 52} },
	{CAAM_EC_CURVE_B_571, {0X44, 1, 72, 72} },
	{CAAM_EC_CURVE_K_163, {0X45, 1, 21, 21} },
	{CAAM_EC_CURVE_K_233, {0X46, 1, 30, 29} },
	{CAAM_EC_CURVE_K_283, {0X47, 1, 36, 36} },
	{CAAM_EC_CURVE_K_409, {0X48, 1, 52, 51} },
	{CAAM_EC_CURVE_K_571, {0X49, 1, 72, 72} },
	{CAAM_EC_CURVE_ECP_GROUP_19, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_ECP_GROUP_20, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_ECP_GROUP_21, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_EC2N_GROUP_3, {0X53, 1, 20, 0} },
	{CAAM_EC_CURVE_EC2N_GROUP_4, {0X54, 1, 24, 0} },
	{CAAM_EC_CURVE_C2PNB163V1, {0X4C, 1, 21, 21} },
	{CAAM_EC_CURVE_C2PNB163V2, {0X4D, 1, 21, 21} },
	{CAAM_EC_CURVE_C2PNB163V3, {0X4E, 1, 21, 21} },
	{CAAM_EC_CURVE_ECPRGF192RANDOM, {0X00, 0, 24, 24} },
	{CAAM_EC_CURVE_ECPRGF224RANDOM, {0X01, 0, 28, 28} },
	{CAAM_EC_CURVE_ECPRGF256RANDOM, {0X02, 0, 32, 32} },
	{CAAM_EC_CURVE_ECPRGF384RANDOM, {0X03, 0, 48, 48} },
	{CAAM_EC_CURVE_ECPRGF521RANDOM, {0X04, 0, 66, 66} },
	{CAAM_EC_CURVE_EC2NGF163RANDOM, {0X40, 1, 21, 21} },
	{CAAM_EC_CURVE_EC2NGF233RANDOM, {0X41, 1, 30, 30} },
	{CAAM_EC_CURVE_EC2NGF283RANDOM, {0X42, 1, 36, 36} },
	{CAAM_EC_CURVE_EC2NGF409RANDOM, {0X43, 1, 52, 52} },
	{CAAM_EC_CURVE_EC2NGF571RANDOM, {0X44, 1, 72, 72} },
	{CAAM_EC_CURVE_EC2NGF163KOBLITZ, {0X45, 1, 21, 21} },
	{CAAM_EC_CURVE_EC2NGF233KOBLITZ, {0X46, 1, 30, 29} },
	{CAAM_EC_CURVE_EC2NGF283KOBLITZ, {0X47, 1, 36, 36} },
	{CAAM_EC_CURVE_EC2NGF409KOBLITZ, {0X48, 1, 52, 51} },
	{CAAM_EC_CURVE_EC2NGF571KOBLITZ, {0X49, 1, 72, 72} },
	{CAAM_EC_CURVE_OAKLEY_3, {0X53, 1, 20, 0} },
	{CAAM_EC_CURVE_OAKLEY_4, {0X54, 1, 24, 0} },
	{CAAM_EC_CURVE_UNDEFINED, {0, 0, 0, 0} }
};

typedef struct {
	u8 *addr_s;
	u8 *addr_f;
	u8 *addr_c;
	u8 *addr_d;
	dma_addr_t phy_addr_s;
	dma_addr_t phy_addr_f;
	dma_addr_t phy_addr_c;
	dma_addr_t phy_addr_d;
	u32 curve_id;
	const ec_curve_t *curve;
	u8 key_color;
	u32 *desc;
} caam_ecdsa_sign_t;

typedef struct {
	u8 *addr_w;
	u8 *addr_f;
	u8 *addr_c;
	u8 *addr_d;
	u8 *addr_tmp;
	dma_addr_t phy_addr_w;
	dma_addr_t phy_addr_f;
	dma_addr_t phy_addr_c;
	dma_addr_t phy_addr_d;
	dma_addr_t phy_addr_tmp;
	u32 curve_id;
	const ec_curve_t *curve;
	u8	key_color;
	u32 *desc;
} caam_ecdsa_verify_t;

typedef struct {
	u8 *addr_s;
	u8 *addr_w;
	dma_addr_t phy_addr_s;
	dma_addr_t phy_addr_w;
	u32 curve_id;
	const ec_curve_t *curve;
	u8	key_color;
	u32 *desc;
} caam_ecdsa_genkey_t;


/**
 * ECDSA (PD=1) Signature Generation PDB
 * @sgf     : scatter-gather field
 * @s_dma   : dma address of private key
 * @f_dma   : dma address of message representative
 * @c_dma   : dma address of signature first half
 * @d_dma   : dma address of signature second half
 */
struct ecdsa_sign_desc_s {
	u32 sgf;
	caam_dma_addr_t s_dma;
	caam_dma_addr_t f_dma;
	caam_dma_addr_t c_dma;
	caam_dma_addr_t d_dma;
} __packed;

/**
 * ECDSA (PD=1) Signature Verification PDB
 * @sgf     : scatter-gather field
 * @w_dma   : dma address of public key
 * @f_dma   : dma address of message representative
 * @c_dma   : dma address of signature first half
 * @d_dma   : dma address of signature second half
 * @tmp_dma : dma address of temporary buffer. CAAM uses this temporary buffer
 *            as internal state buffer. It is assumed to be as long as w.
 */
struct ecdsa_verify_desc_s {
	u32 sgf;
	caam_dma_addr_t w_dma;
	caam_dma_addr_t f_dma;
	caam_dma_addr_t c_dma;
	caam_dma_addr_t d_dma;
	caam_dma_addr_t tmp_dma;
} __packed;

/**
 * ECDSA (PD=1) ECC key pair generation PDB
 * @sgf     : scatter-gather field
 * @s_dma   : dma address of private key
 * @w_dma   : dma address of public key
 */
struct ecdsa_genkey_desc_s {
	u32 sgf;
	caam_dma_addr_t s_dma;
	caam_dma_addr_t w_dma;
} __packed;

/* pk per-device context */
struct caam_ctx {
	struct device *jrdev;
};

struct caam_operation_result {
	struct completion completion;
	int err;
};

static struct caam_ctx *caam_ecdsa_ctx;

void caam_ecdsa_verify_jobdesc(u32 *desc,
	caam_ecdsa_verify_t *ecdsa_verify);
void caam_ecdsa_sign_jobdesc(u32 *desc,
	caam_ecdsa_sign_t *ecdsa_sign);
void caam_ecdsa_genkey_jobdesc(u32 *desc,
	caam_ecdsa_genkey_t *ecdsa_genkey);
int caam_ecdsa_verify(caam_ecdsa_verify_t *ecdsa_verify);
int caam_ecdsa_verify_deinit(caam_ecdsa_verify_t *ecdsa_verify);
int caam_ecdsa_verify_init(caam_ecdsa_verify_t *ecdsa_verify);
int caam_ecdsa_sign(caam_ecdsa_sign_t *ecdsa_sign);
int caam_ecdsa_sign_deinit(caam_ecdsa_sign_t *ecdsa_sign);
int caam_ecdsa_sign_init(caam_ecdsa_sign_t *ecdsa_sign);
int caam_ecdsa_genkey_init(caam_ecdsa_genkey_t *ecdsa_genkey);
int caam_ecdsa_genkey_deinit(caam_ecdsa_genkey_t *ecdsa_genkey);
int caam_ecdsa_genkey(caam_ecdsa_genkey_t *ecdsa_genkey);
void caam_operation_done(struct device *dev, u32 *desc, u32 err,
	void *context);
const ec_curve_t *caam_select_ec_curve(int cid);

struct device *caam_ecdsa_get_jrdev(void)
{
	if (caam_ecdsa_ctx != NULL) {
		if (caam_ecdsa_ctx->jrdev != NULL) {
			return caam_ecdsa_ctx->jrdev;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(caam_ecdsa_get_jrdev);

int caam_pk_status(void)
{
	return NULL != caam_ecdsa_ctx ? 1:0;
}
EXPORT_SYMBOL(caam_pk_status);


const ec_curve_t *caam_select_ec_curve(int cid)
{
	const ec_curve_t *curve = &caam_ec_curve_list[0];

	for (; curve != &(caam_ec_curve_list[sizeof(caam_ec_curve_list)/
		sizeof(ec_curve_t)]); curve++)
	if (curve->cid == cid)
		return curve;
	return NULL;
}

int caam_ecdsa_sign_init(caam_ecdsa_sign_t *ecdsa_sign)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	const ec_curve_data_t *curve_data;
	int ret = 0;
	size_t total_len;

	ecdsa_sign->curve = caam_select_ec_curve(ecdsa_sign->curve_id);
	if (unlikely(!ecdsa_sign->curve))
		goto curve_select_fail;

	curve_data = &ecdsa_sign->curve->data;

	ecdsa_sign->desc = kmalloc(MAX_CAAM_DESCSIZE *
					sizeof(u32), GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_sign->desc))
		goto desc_alloc_fail;
	total_len = curve_data->l_len + curve_data->n_len * 3;
	ecdsa_sign->addr_s = dma_alloc_coherent(jrdev, total_len,
				&ecdsa_sign->phy_addr_s, GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_sign->addr_s))
		goto q_alloc_fail;

	memset(ecdsa_sign->addr_s, 0, total_len);
	ecdsa_sign->addr_f = ecdsa_sign->addr_s + curve_data->l_len;
	ecdsa_sign->phy_addr_f = ecdsa_sign->phy_addr_s + curve_data->l_len;
	ecdsa_sign->addr_c = ecdsa_sign->addr_f + curve_data->n_len;
	ecdsa_sign->phy_addr_c = ecdsa_sign->phy_addr_f + curve_data->n_len;
	ecdsa_sign->addr_d = ecdsa_sign->addr_c + curve_data->n_len;
	ecdsa_sign->phy_addr_d = ecdsa_sign->phy_addr_c + curve_data->n_len;

	return ret;

q_alloc_fail:
	kfree(ecdsa_sign->desc);
desc_alloc_fail:
	return -ENOMEM;
curve_select_fail:
	return -EINVAL;
}
EXPORT_SYMBOL(caam_ecdsa_sign_init);

int caam_ecdsa_sign_deinit(caam_ecdsa_sign_t *ecdsa_sign)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;

	const ec_curve_data_t *curve_data = &ecdsa_sign->curve->data;

	dma_free_coherent(jrdev, curve_data->l_len + curve_data->n_len * 3,
	(void *)ecdsa_sign->addr_s, ecdsa_sign->phy_addr_s);

	kfree(ecdsa_sign->desc);

	return 0;
}
EXPORT_SYMBOL(caam_ecdsa_sign_deinit);

int caam_ecdsa_sign(caam_ecdsa_sign_t *ecdsa_sign)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	u32 *desc = ecdsa_sign->desc;
	int ret = 0;
	struct caam_operation_result res;

	const ec_curve_data_t *curve_data = &ecdsa_sign->curve->data;

	memset(desc, 0, MAX_CAAM_DESCSIZE * sizeof(u32));

	caam_ecdsa_sign_jobdesc(desc, ecdsa_sign);

	res.err = 0;
	init_completion(&res.completion);

	ret = caam_jr_enqueue(jrdev, desc, caam_operation_done, &res);
	if (!ret) {
		wait_for_completion(&res.completion);
		ret = res.err;
	}

	dma_sync_single_for_cpu(jrdev, ecdsa_sign->phy_addr_c,
					curve_data->n_len, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(jrdev, ecdsa_sign->phy_addr_d,
					curve_data->n_len, DMA_FROM_DEVICE);

	return ret;
}
EXPORT_SYMBOL(caam_ecdsa_sign);

int caam_ecdsa_verify_init(caam_ecdsa_verify_t *ecdsa_verify)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	const ec_curve_data_t *curve_data;
	int ret = 0;
	size_t total_len;

	ecdsa_verify->curve = caam_select_ec_curve(ecdsa_verify->curve_id);
	if (unlikely(!ecdsa_verify->curve))
		goto curve_select_fail;

	curve_data = &ecdsa_verify->curve->data;
	ecdsa_verify->desc = kmalloc(MAX_CAAM_DESCSIZE *
				sizeof(u32), GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_verify->desc))
		goto desc_alloc_fail;
	total_len = curve_data->l_len * 4 + curve_data->n_len * 3;
	ecdsa_verify->addr_w = dma_alloc_coherent(jrdev, total_len,
		&ecdsa_verify->phy_addr_w, GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_verify->addr_w))
		goto q_alloc_fail;

	memset(ecdsa_verify->addr_w, 0, total_len);
	ecdsa_verify->addr_f = ecdsa_verify->addr_w + curve_data->l_len * 2;
	ecdsa_verify->phy_addr_f  = ecdsa_verify->phy_addr_w +
			curve_data->l_len * 2;
	ecdsa_verify->addr_c = ecdsa_verify->addr_f + curve_data->n_len;
	ecdsa_verify->phy_addr_c  = ecdsa_verify->phy_addr_f +
			curve_data->n_len;
	ecdsa_verify->addr_d = ecdsa_verify->addr_c + curve_data->n_len;
	ecdsa_verify->phy_addr_d  = ecdsa_verify->phy_addr_c +
			curve_data->n_len;
	ecdsa_verify->addr_tmp = ecdsa_verify->addr_d + curve_data->n_len;
	ecdsa_verify->phy_addr_tmp  = ecdsa_verify->phy_addr_d +
			curve_data->n_len;

	return ret;

q_alloc_fail:
	kfree(ecdsa_verify->desc);
desc_alloc_fail:
	return -ENOMEM;
curve_select_fail:
	return -EINVAL;
}
EXPORT_SYMBOL(caam_ecdsa_verify_init);

int caam_ecdsa_verify_deinit(caam_ecdsa_verify_t *ecdsa_verify)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	const ec_curve_data_t *curve_data = &ecdsa_verify->curve->data;

	dma_free_coherent(jrdev, curve_data->l_len * 2 +
			curve_data->n_len * 3,
			(void *)ecdsa_verify->addr_w,
			ecdsa_verify->phy_addr_w);
	kfree(ecdsa_verify->desc);

	return 0;
}
EXPORT_SYMBOL(caam_ecdsa_verify_deinit);


int caam_ecdsa_verify(caam_ecdsa_verify_t *ecdsa_verify)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	u32 *desc = ecdsa_verify->desc;

	int ret = 0;
	struct caam_operation_result res;

	memset(desc, 0, MAX_CAAM_DESCSIZE * sizeof(u32));

	caam_ecdsa_verify_jobdesc(desc, ecdsa_verify);

	res.err = 0;
	init_completion(&res.completion);

	/*If the signature is correct, caam_jr_enqueue terminates normally.*/
	ret = caam_jr_enqueue(jrdev, desc, caam_operation_done, &res);
	if (!ret) {
		wait_for_completion(&res.completion);
	}

	if (res.err == 0)
		return ECDSA_VERIFY_SUCCESS;
	else if ((res.err & 0xff) == ECDSA_INVA_SIG_STATUS)
		return ECDSA_VERIFY_FAIL;
	else
		return ret;
}
EXPORT_SYMBOL(caam_ecdsa_verify);

int caam_ecdsa_genkey_init(caam_ecdsa_genkey_t *ecdsa_genkey)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	const ec_curve_data_t *curve_data = NULL;
	int ret = 0;
	size_t total_len;

	ecdsa_genkey->curve = caam_select_ec_curve(ecdsa_genkey->curve_id);
	if (unlikely(!ecdsa_genkey->curve))
		goto curve_select_fail;

	curve_data = &ecdsa_genkey->curve->data;

	ecdsa_genkey->desc = kmalloc(MAX_CAAM_DESCSIZE *
		sizeof(u32), GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_genkey->desc))
		goto desc_alloc_fail;
	total_len = curve_data->l_len * 3;
	ecdsa_genkey->addr_s = dma_alloc_coherent(jrdev, total_len,
		&ecdsa_genkey->phy_addr_s, GFP_KERNEL | GFP_DMA);
	if (unlikely(!ecdsa_genkey->addr_s))
		goto q_alloc_fail;

	memset(ecdsa_genkey->addr_s, 0, total_len);
	ecdsa_genkey->addr_w = ecdsa_genkey->addr_s + curve_data->l_len;
	ecdsa_genkey->phy_addr_w = ecdsa_genkey->phy_addr_s + curve_data->l_len;

	return ret;

q_alloc_fail:
	kfree(ecdsa_genkey->desc);
desc_alloc_fail:
	return -ENOMEM;
curve_select_fail:
	return -EINVAL;
}
EXPORT_SYMBOL(caam_ecdsa_genkey_init);

int caam_ecdsa_genkey_deinit(caam_ecdsa_genkey_t *ecdsa_genkey)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	const ec_curve_data_t *curve_data = NULL;

	curve_data = &ecdsa_genkey->curve->data;

	dma_free_coherent(jrdev, curve_data->l_len * 3,
	(void *)ecdsa_genkey->addr_s, ecdsa_genkey->phy_addr_s);

	kfree(ecdsa_genkey->desc);

	return 0;
}
EXPORT_SYMBOL(caam_ecdsa_genkey_deinit);

int caam_ecdsa_genkey(caam_ecdsa_genkey_t *ecdsa_genkey)
{
	struct device *jrdev = caam_ecdsa_ctx->jrdev;
	u32 *desc = ecdsa_genkey->desc;
	const ec_curve_data_t *curve_data = &ecdsa_genkey->curve->data;

	int ret = 0;
	struct caam_operation_result res;

	memset(desc, 0, MAX_CAAM_DESCSIZE * sizeof(u32));

	caam_ecdsa_genkey_jobdesc(desc, ecdsa_genkey);

	res.err = 0;
	init_completion(&res.completion);

	ret = caam_jr_enqueue(jrdev, desc, caam_operation_done, &res);
	if (!ret) {
		wait_for_completion(&res.completion);
		ret = res.err;
	}

	dma_sync_single_for_cpu(jrdev, ecdsa_genkey->phy_addr_s,
		curve_data->l_len, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(jrdev, ecdsa_genkey->phy_addr_w,
		curve_data->l_len*2, DMA_FROM_DEVICE);

	return ret;
}
EXPORT_SYMBOL(caam_ecdsa_genkey);


void caam_ecdsa_sign_jobdesc(u32 *desc, caam_ecdsa_sign_t *ecdsa_sign)
{
	const ec_curve_t *curve = caam_select_ec_curve(ecdsa_sign->curve_id);
	const ec_curve_data_t *curve_data = &curve->data;
	u32 cmd;

	init_job_desc_pdb(desc, 0, sizeof(struct ecdsa_sign_desc_s));
	append_cmd(desc, ((curve_data->caam_ec_id & 0x7F) << 7) |
				CAAM_ECDSA_PD);
	append_ptr(desc, ecdsa_sign->phy_addr_s);
	append_ptr(desc, ecdsa_sign->phy_addr_f);
	append_ptr(desc, ecdsa_sign->phy_addr_c);
	append_ptr(desc, ecdsa_sign->phy_addr_d);
	cmd  = CAAM_PROTOP_CTYPE | OP_TYPE_UNI_PROTOCOL |
			OP_PCLID_DSASIGN | OP_PCL_PKPROT_ECC;
	cmd |= (ecdsa_sign->key_color ==
		KEY_COLOR_BLACK) ? CAAM_PROTINFO_SEC_KEY:0;
	append_operation(desc, cmd);

	dma_sync_single_for_device(caam_ecdsa_ctx->jrdev,
		ecdsa_sign->phy_addr_s, curve_data->l_len * 2 +
		curve_data->n_len * 2, DMA_TO_DEVICE);
}

void caam_ecdsa_verify_jobdesc(u32 *desc, caam_ecdsa_verify_t *ecdsa_verify)
{
	const ec_curve_t *curve = caam_select_ec_curve(ecdsa_verify->curve_id);
	const ec_curve_data_t *curve_data = &curve->data;

	init_job_desc_pdb(desc, 0, sizeof(struct ecdsa_verify_desc_s));
	append_cmd(desc, ((curve_data->caam_ec_id & 0x7F) << 7) |
			CAAM_ECDSA_PD);
	append_ptr(desc, ecdsa_verify->phy_addr_w);
	append_ptr(desc, ecdsa_verify->phy_addr_f);
	append_ptr(desc, ecdsa_verify->phy_addr_c);
	append_ptr(desc, ecdsa_verify->phy_addr_d);
	append_ptr(desc, ecdsa_verify->phy_addr_tmp);
	append_operation(desc, CAAM_PROTOP_CTYPE | OP_TYPE_UNI_PROTOCOL |
			OP_PCLID_DSAVERIFY | OP_PCL_PKPROT_ECC);

	dma_sync_single_for_device(caam_ecdsa_ctx->jrdev,
					ecdsa_verify->phy_addr_w,
	curve_data->l_len * 4 + curve_data->n_len * 3, DMA_TO_DEVICE);
}

void caam_ecdsa_genkey_jobdesc(u32 *desc, caam_ecdsa_genkey_t *ecdsa_genkey)
{
	u32 cmd;
	const ec_curve_t *curve = caam_select_ec_curve(ecdsa_genkey->curve_id);
	const ec_curve_data_t *curve_data = &curve->data;

	init_job_desc_pdb(desc, 0, sizeof(struct ecdsa_genkey_desc_s));
	append_cmd(desc, ((curve_data->caam_ec_id & 0x7F)  << 7) |
					CAAM_CMD_ECC_GEN_KP);
	append_ptr(desc, ecdsa_genkey->phy_addr_s);
	append_ptr(desc, ecdsa_genkey->phy_addr_w);
	cmd  = CAAM_PROTOP_CTYPE | OP_TYPE_UNI_PROTOCOL |
			 OP_PCLID_PUBLICKEYPAIR | OP_PCL_PKPROT_ECC;
	cmd |= (ecdsa_genkey->key_color == KEY_COLOR_BLACK) ?
			CAAM_PROTINFO_SEC_KEY:0;
	append_operation(desc, cmd);

	dma_sync_single_for_device(caam_ecdsa_ctx->jrdev,
	ecdsa_genkey->phy_addr_s, curve_data->l_len * 3, DMA_TO_DEVICE);
}

void caam_operation_done(struct device *dev, u32 *desc, u32 err, void *context)
{
	struct caam_operation_result *res = context;
#ifdef DEBUG
	dev_err(dev, "%s %d: err 0x%x\n", __func__, __LINE__, err);
	if (err)
		caam_jr_strstatus(dev, err);
#endif
	res->err = err;
	complete(&res->completion);
}

/* Public Key Cryptography module initialization handler */
static int __init caam_ecdsa_init(void)
{
	struct device *jrdev;
	struct device_node *dev_node;
	struct platform_device *pdev;
	struct device *ctrldev;
	struct caam_drv_private *priv;
	u32 cha_inst, pk_inst;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev) {
		of_node_put(dev_node);
		return -ENODEV;
	}

	ctrldev = &pdev->dev;

	priv = dev_get_drvdata(ctrldev);

	of_node_put(dev_node);

	/*
	 * If priv is NULL, it's probably because the caam driver wasn't
	 * properly initialized (e.g. RNG4 init failed). Thus, bail out here.
	 */
	if (!priv)
		return -ENODEV;

	/* Determine public key hardware accelerator presence. */
	if (priv->has_seco) {
		int i = priv->first_jr_index;

		cha_inst = rd_reg32(&priv->jr[i]->perfmon.cha_num_ls);
	} else {
		cha_inst = rd_reg32(&priv->ctrl->perfmon.cha_num_ls);
	}
	pk_inst = (cha_inst & CHA_ID_LS_PK_MASK) >> CHA_ID_LS_PK_SHIFT;

	/* Do not register functions if PKHA is not present. */
	if (!pk_inst)
		return -ENODEV;

	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev)) {
		caam_ecdsa_ctx = NULL;
		pr_err("Job Ring Device allocation for transform failed\n");
		return PTR_ERR(jrdev);
	}
	caam_ecdsa_ctx = kmalloc(sizeof(struct caam_ctx), GFP_DMA | GFP_KERNEL);
	if (unlikely(!caam_ecdsa_ctx))
		return -ENOMEM;

	caam_ecdsa_ctx->jrdev = jrdev;

	return 0;
}

static void __exit caam_ecdsa_exit(void)
{
	caam_jr_free(caam_ecdsa_ctx->jrdev);
	kfree(caam_ecdsa_ctx);
	caam_ecdsa_ctx = NULL;
}

module_init(caam_ecdsa_init);
module_exit(caam_ecdsa_exit);
MODULE_DESCRIPTION("CAAM ECDSA driver with Black Key support");
MODULE_LICENSE("Dual BSD/GPL");
