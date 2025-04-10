#include <iostream>

using namespace std;

typedef struct{
	double
	ini_angka = 0.00;
}random_variable_t;

random_variable_t variable;

double check_angka_tanpa_pointer(random_variable var){
	var.ini_angka = 20.00;
	return var.ini_angka;
}

double check_angka_dengan_pointer(random_variable *var){
	var->ini_angka = 20.00;
	return var->ini_angka;
}

double angka_tanpa_pointer;
double angka_dengan_pointer;

int main(void){
	random_variable variable;
	
	angka_tanpa_pointer = check_angka_tanpa_pointer(variable);
	cout << "Tanpa pointer: " << endl;
	cout << angka_tanpa_pointer  << endl;
	
	angka_dengan_pointer = check_angka_dengan_pointer(&variable);
	cout << "Dengan pointer: " << endl;
	cout << angka_dengan_pointer;
	
	return 0;
}