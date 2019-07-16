#ifndef _OCTO_H
#define _OCTO_H

#define octoNO_OPTIMIZATION

#define octoMOTOR_A_INDEX           ( 0 )
#define octoMOTOR_B_INDEX           ( 1 )
#define octoMOTOR_C_INDEX           ( 2 )
#define octoMOTOR_D_INDEX           ( 3 )
#define octoMOTOR_E_INDEX           ( 4 )
#define octoMOTOR_F_INDEX           ( 5 )
#define octoMOTOR_INDEX_INCREMENT   ( 1 )

#define octoARM_A_INDEX             ( 0 )
#define octoARM_B_INDEX             ( 1 )
#define octoARM_C_INDEX             ( 2 )
#define octoARM_D_INDEX             ( 3 )
#define octoARM_E_INDEX             ( 4 )
#define octoARM_F_INDEX             ( 5 )
#define octoARM_G_INDEX             ( 6 )
#define octoARM_H_INDEX             ( 7 )
#define octoARM_INDEX_INCREMENT     ( 1 )

#define octoRPM                     ( 15 )

#define octoA0                      ( eArm0 )
#define octoA1                      ( eArm1 )
#define octoA2                      ( eArm2 )
#define octoA3                      ( eArm3 )
#define octoA4                      ( eArm4 )
#define octoA5                      ( eArm5 )
#define octoA6                      ( eArm6 )
#define octoA7                      ( eArm7 )

#define octoT0                      ( ePlaceholder0 )
#define octoT1                      ( ePlaceholder1 )
#define octoT2                      ( ePlaceholder2 )
#define octoT3                      ( ePlaceholder3 )
#define octoT4                      ( ePlaceholder4 )
#define octoT5                      ( ePlaceholder5 )
#define octoT6                      ( ePlaceholder6 )
#define octoT7                      ( ePlaceholder7 )
#define octoF0                      ( ePlaceholder8 )
#define octoF1                      ( ePlaceholder9 )
#define octoF2                      ( ePlaceholder10 )
#define octoF3                      ( ePlaceholder11 )

typedef enum {
    eArm0 = 0,
    eArm1,
    eArm2,
    eArm3,
    eArm4,
    eArm5,
    eArm6,
    eArm7,
} eArm;

typedef enum {
    ePlaceholder0 = 0,
    ePlaceholder1,
    ePlaceholder2,
    ePlaceholder3,
    ePlaceholder4,
    ePlaceholder5,
    ePlaceholder6,
    ePlaceholder7,
    ePlaceholder8,
    ePlaceholder9,
    ePlaceholder10,
    ePlaceholder11,
} ePlaceholder;

typedef struct xDISPLACE_INFORMATION {
	ePlaceholder ePlaceholderFrom;
	ePlaceholder ePlaceholderTo;
	unsigned char ucArm;
	unsigned short int usFirstRotationInDegrees;
	unsigned short int usSecondRotationInDegrees;
} DisplaceInformation_t;

#endif /* _OCTO_H */

