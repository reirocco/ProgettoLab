/*
 * json.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Rei
 */


#include "main.h"
#include "string.h"

#define SHIFT 100

const char SEPARATOR = ':';
const char COMMA = ',';
char* ptr;
int initial_number = 100;
int last = 0;


void json_init();
void extend();
void addElement(char (*key)[], int value);
char* getJSON();
void increment_last(int addend);




void json_init(){
	// Dynamically allocate memory using calloc()
    ptr = (char*)calloc(initial_number, sizeof(char));

    // Check if the memory has been successfully
	// allocated by malloc or not
	if (ptr == NULL) {
	   printf("Memory not allocated.\r\n");
	   exit(0);
	}

	strcat(ptr, '{');
	last = 2;
}

void extend(){
	ptr = realloc(ptr, initial_number + SHIFT * sizeof(int));
}

void addElement(char (*key)[], int value){
	strcat(ptr, key);

	strcat(ptr, SEPARATOR);

	strcat(ptr,atoi(value));

	strcat(ptr,COMMA);
}


char* getJSON(){
	strcat(ptr,'}');
	//strcat(ptr,'\0');
	strcat(ptr,'\r\n');
	return ptr;
}

void json_reset(){
	json_init();
}

