#include "FreeRTOS.h"
#include "client.h"
#include "stm32f103xb.h"
#include "ax.h"
#include "octo.h"

typedef struct xDISPLACE_INFORMATION {
	ePlaceholder ePlaceholderFrom;
	ePlaceholder ePlaceholderTo;
	unsigned char ucArm;
	unsigned short int usFirstRotationInDegrees;
	unsigned short int usSecondRotationInDegrees;
} DisplaceInformation_t;

static BaseType_t prvDoColoursMatch(BaseType_t xColourA, BaseType_t xColourB);
static void prvGetDisplaceInformation( ePlaceholder ePlaceholdersFrom[4], ePlaceholder ePlaceholdersTo[4], DisplaceInformation_t xDisplaceInformation[64] );

void vTaskClient( void * pvParameters )
{
BaseType_t xPlaceholderColoursFirstRound[12];
BaseType_t xPlaceholderColoursSecondRound[12];
BaseType_t xPlaceholderColourFirst;
BaseType_t xPlaceholderColourSecond;
BaseType_t xMatchingPlaceholderColour;
BaseType_t xPlaceholdersFrom[4];
BaseType_t xPlaceholdersTo[4];
BaseType_t xPlaceholdersIndex;
BaseType_t xEmptyPlaceholderColour;
DisplaceInformation_t xDisplaceInformation[64];
    while (1)
    {
        //Wait for button press.
        while (GPIOB->IDR & GPIO_IDR_IDR8);
        while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            //Receive response.
            //Process response.
            xPlaceholderColoursFirstRound[ePlaceholder] = 0;
        }
        //Wait for button press.
        while (GPIOB->IDR & GPIO_IDR_IDR8);
        while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            //Receive response.
            //Process response.
            xPlaceholderColoursSecondRound[ePlaceholder] = 0;
        }
        //Terminate if there are balls with a colour that do not match any of the first reading.
        for (ePlaceholder ePlaceholderSecond = ePlaceholder0; ePlaceholderSecond <= ePlaceholder11; ++ePlaceholderSecond)
        {
            xPlaceholderColourSecond = xPlaceholderColoursSecondRound[ePlaceholderSecond];
            if (prvDoColoursMatch(xPlaceholderColourSecond, xEmptyPlaceholderColour))
            {
                continue;
            }
            xMatchingPlaceholderColour = 0;
            for (ePlaceholder ePlaceholderFirst = ePlaceholder0; ePlaceholderFirst <= ePlaceholder11; ++ePlaceholderFirst)
            {
                xPlaceholderColourFirst = xPlaceholderColoursFirstRound[ePlaceholderFirst];
                if (prvDoColoursMatch(xPlaceholderColourFirst, xEmptyPlaceholderColour))
                {
                    continue;
                }
                if (prvDoColoursMatch(xPlaceholderColourFirst, xPlaceholderColourSecond))
                {
                    xPlaceholdersFrom[xPlaceholdersIndex] = ePlaceholderSecond;
                    xPlaceholdersTo[xPlaceholdersIndex] = ePlaceholderFirst;
                    ++xPlaceholdersIndex;
                    xMatchingPlaceholderColour = 1;
                    break;
                }
            }
            if (!xMatchingPlaceholderColour)
            {
                //Terminate.
            }
        }
        //Determine edges based on first and second reading.
        //While all edges have not been processed:
            //Go through all edges and buffer as many movements as possible in a round.
            //Execute round.
            //Check if balls have reached their destination placeholders, otherwise halt system or wait until user has manually placed runaway balls to their destination placeholders.
        //Done.
    }
}

static BaseType_t prvDoColoursMatch(BaseType_t xColourA, BaseType_t xColourB)
{
    if (xColourA == xColourB)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void prvGetDisplaceInformation( ePlaceholder ePlaceholdersFrom[4], ePlaceholder ePlaceholdersTo[4], DisplaceInformation_t xDisplaceInformation[64] )
{

}

