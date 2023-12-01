/*
 * gui.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Juan Felipe Ordóñez
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include <stdint.h>
#include <stdio.h>
#include "main.h"

// Enum declaration for representing gender
enum Gender {
    M,  // Represents Male
    F   // Represents Female
};

// Function prototypes
void displayResult(const unsigned char* resultType);
void print_Gender_Age(enum Gender gender, int age);
void print_BPM(int bpm);
void print_Result(int age, int bpm, enum Gender gender);

#endif /* INC_GUI_H_ */
