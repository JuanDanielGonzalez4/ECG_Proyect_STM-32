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
void assess_Patient_Health(enum Gender gender, int age, int bpm);

#endif /* INC_GUI_H_ */
