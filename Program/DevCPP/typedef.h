#ifndef TYPEDEF_H
#define TYPEDEF_H

typedef struct{
	double
	ini_angka = 20.00;
	
	char
	ini_huruf = 'A';
}random_variable;

double check_angka_dengan_pointer(random_variable *var);
double check_angka_tanpa_pointer(random_variable var);

#endif
