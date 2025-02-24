#include "image.h"

#include <stdlib.h>
#include <stdio.h>

/*
 * Loads a PPM image
 */
void loadImage(const char* filename, int* imageWidth, int* imageHeight, unsigned char** imageData)
{
	// the ID of the image file
	FILE* fileID;

	// maxValue
	int  maxValue;

	// total number of pixels in the image
	int  totalPixels;

	// temporary character
	char tempChar;

	// counter variable for the current pixel in the image
	int i;

	// array for reading in header information
	char headerLine[100];

	// if the original values are larger than 255
	float RGBScaling;

	// temporary variables for reading in the red, green and blue data of each pixel
	char red, green, blue;

	int filePos = 0;
	int bufferPos;

	// open the image file for reading - note this is hardcoded would be better to provide a parameter which
	//is the file name. There are 3 PPM files you can try out mount03, sky08 and sea02.
	errno_t err = fopen_s(&fileID, filename, "r");

	if (err != 0)
	{
		printf("Failed to load image %s.", filename);
		exit(0);
	}

	fseek(fileID, 0, SEEK_END);
	long fileSize = ftell(fileID);
	rewind(fileID);

	char* buffer = calloc(1, fileSize);

	if (buffer == NULL)
	{
		printf("Failed to allocate image file data buffer.");
		exit(0);
	}

	fread(buffer, fileSize, 1, fileID);
	fclose(fileID);

	// read in the first header line
	//    - "%[^\n]"  matches a string of all characters not equal to the new line character ('\n')
	//    - so we are just reading everything up to the first line break
	sscanf_s(buffer, "%[^\n] %n", headerLine, sizeof(headerLine), &bufferPos);
	filePos += bufferPos;

	// make sure that the image begins with 'P3', which signifies a PPM file
	if ((headerLine[0] != 'P') || (headerLine[1] != '6'))
	{
		printf("This is not a PPM file!\n %c", headerLine[1]);
		exit(0);
	}

	// we have a PPM file
	printf("This is a PPM file\n");

	// read in the first character of the next line
	sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	filePos += bufferPos;

	// while we still have comment lines (which begin with #)
	while (tempChar == '#')
	{
		// read in the comment
		sscanf_s(&buffer[filePos], "%[^\n] %n", headerLine, sizeof(headerLine), &bufferPos);
		filePos += bufferPos;

		// print the comment
		printf("%s\n", headerLine);

		// read in the first character of the next line
		sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	}

	// read in the image hieght, width and the maximum value
	sscanf_s(&buffer[filePos], "%d %d %d%n", imageWidth, imageHeight, &maxValue, &bufferPos);
	filePos += bufferPos;

	sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	filePos += bufferPos;

	// print out the information about the image file
	printf("%d rows  %d columns  max value= %d\n", *imageHeight, *imageWidth, maxValue);

	// compute the total number of pixels in the image
	totalPixels = (*imageWidth) * (*imageHeight);

	// allocate enough memory for the image  (3*) because of the RGB data
	*imageData = calloc(totalPixels, 3 * sizeof(unsigned char));

	if (*imageData == NULL)
	{
		printf("Failed to allocate image data buffer.");
		exit(0);
	}

	// determine the scaling for RGB values
	RGBScaling = 255.0f / maxValue;


	// if the maxValue is 255 then we do not need to scale the 
	//    image data values to be in the range or 0 to 255
	if (maxValue == 255)
	{
		for (i = 0; i < totalPixels; i++)
		{
			// read in the current pixel from the file
			red = buffer[filePos++];
			green = buffer[filePos++];
			blue = buffer[filePos++];

			// store the red, green and blue data of the current pixel in the data array
			(*imageData)[3 * totalPixels - 3 * i - 3] = red;
			(*imageData)[3 * totalPixels - 3 * i - 2] = green;
			(*imageData)[3 * totalPixels - 3 * i - 1] = blue;

			//(*imageData)[3 * i] = red;
			//(*imageData)[(3 * i) + 1] = green;
			//(*imageData)[(3 * i) + 2] = blue;
		}
	}
	else  // need to scale up the data values
	{
		for (i = 0; i < totalPixels; i++)
		{
			// read in the current pixel from the file
			red = buffer[filePos++];
			green = buffer[filePos++];
			blue = buffer[filePos++];

			// store the red, green and blue data of the current pixel in the data array
			(*imageData)[3 * totalPixels - 3 * i - 3] = (unsigned char)(red * RGBScaling);
			(*imageData)[3 * totalPixels - 3 * i - 2] = (unsigned char)(green * RGBScaling);
			(*imageData)[3 * totalPixels - 3 * i - 1] = (unsigned char)(blue * RGBScaling);
		}
	}


	// close the image file
	fclose(fileID);
}