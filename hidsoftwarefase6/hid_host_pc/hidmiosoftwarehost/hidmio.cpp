// hidmio.cpp : Defines the entry point for the console application.
//
//para Microsoft Visual C++ a partir de versión 2005, 
//necesita DDK
//EJEMPLO MINIMO (separar en funciones y completarlo para 
//cuando hay varios dispositivos HID)

#include "stdafx.h"
#include <objbase.h>
extern "C"
{
	#include <hidsdi.h>
	#include <setupapi.h>
}

#include <conio.h>
//#include <wtypes.h>



int main(int argc, char* argv[])
{
//para pasarlos a readfile y writefile, podemos tener dos arrays diferentes
unsigned	char datos[20];
     unsigned long bytesleidos;

GUID mio;

HANDLE handleclase,handledisposi;

SP_DEVICE_INTERFACE_DATA infodisposi;

PSP_DEVICE_INTERFACE_DETAIL_DATA infocamino;

//suponemos que solo hay un dispositivo HID o que el nuestro es el primero 0

int numlista=4;

unsigned long longitud=0,requerido;

//para chequear errores
int resultado=1;


//obtenemos el GUID de la clase HID
HidD_GetHidGuid(&mio);

//obtenemos un manejador de la clase
handleclase=SetupDiGetClassDevs(&mio,NULL,NULL, DIGCF_PRESENT|DIGCF_INTERFACEDEVICE);

infodisposi.cbSize=sizeof(infodisposi);
//obtenemos la informacion del primer dispositivo HID numlista=0
//En lo que sigue suponemos que solo hay un dispositivo HID,
//en caso contrario habria que recorrer numlista= 1, 2, 3, ....
//comprobando el idvendedor idproducto sabriamos cual es nuestro dispositivo

resultado=SetupDiEnumDeviceInterfaces(handleclase,0,&mio,numlista,&infodisposi);

if(resultado==0)
printf("No detectados dispositivos\n");

//comentado metodo "elegante" de obtener longitud de infocamino

/* resultado=SetupDiGetDeviceInterfaceDetail(handleclase,&infodisposi,NULL,0,&longitud,NULL);
printf("Longitud= %d\n",longitud);
infocamino= (PSP_DEVICE_INTERFACE_DETAIL_DATA) malloc(longitud);*/


//metodo "no elegante" donde esta 500 longitud
infocamino= (PSP_DEVICE_INTERFACE_DETAIL_DATA) malloc(500);
infocamino->cbSize =sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

//obtenemos la informacion del camino y nombre
resultado=SetupDiGetDeviceInterfaceDetail(handleclase,&infodisposi,infocamino,500,&requerido,NULL);

if(resultado==0)
printf("No conseguido camino\n");
printf("%s \n", infocamino->DevicePath); //meramente informativo
//abrimos el dispositivo
//HID\VID_0539&PID_5BA0\6&2200464&0&0000

handledisposi=CreateFile("\\\\?\\HID#VID_0539&PID_5BA0#6&2200464&0&0000#{4D1E55B2-F16F-11CF-88CB-001111000030}",
							GENERIC_READ|GENERIC_WRITE,
							 FILE_SHARE_READ|FILE_SHARE_WRITE,
							 NULL,
							 OPEN_EXISTING,
							 0,
							 NULL);

if(handledisposi==INVALID_HANDLE_VALUE)
printf("Error abriendo dispositivo\n");

else

while(1)
{
	//alternar si queremos leer  en el dispo
	ReadFile(handledisposi,datos,7,&bytesleidos,NULL);
	printf("datos= %d,%d,%d,%d,%d,%d, %d\n",datos[0],datos[1],datos[2],datos[3],datos[4],datos[5], bytesleidos);
	INT16 hum= (datos[2] << 8 | datos[1]);
	INT16 temp = (datos[4] << 8 | datos[3]);
	float hum2 = hum / 10.0;
	float temp2 = temp / 10.0;
	printf("hum : %.1f \n", hum2);
	printf("temp : %.1f \n",temp2);
	//si queremos enviar  al dispositivo 
	//float ang = 90;
	//datos[1]= 90;
	//datos[0]=0;
//resultado=	WriteFile(handledisposi,datos,7,&bytesleidos,NULL);

printf("datos= %d,%d,%d,%d, %d\n",datos[0],datos[1],datos[2],resultado,bytesleidos);

  }	
	
	return 0;
}

