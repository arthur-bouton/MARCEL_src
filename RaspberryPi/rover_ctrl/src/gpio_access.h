#ifndef GPIO_ACCESS_H
#define GPIO_ACCESS_H

#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>

 
// Définir le GPIO g en tant qu'entrée :
// (toujours utiliser INP_GPIO(g) avant OUT_GPIO(g) ou SET_GPIO_ALT(g,f))
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))

// Définir le GPIO g en tant que sortie :
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

// Définir la fonction du GPIO g :
#define SET_GPIO_ALT(g,f) *(gpio+(((g)/10))) |= (((f)<=3?(f)+4:(f)==4?3:2)<<(((g)%10)*3))
 
// Mettre à 1 ou à 0 le GPIO correspondant au poids du bit à 1. Les 0 sont ignorés.
#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)
 
// Accéder à la valeur du GPIO g :
// (retourne ( 1 << g ) si à 3.3V, 0 si à 0V )
#define GET_GPIO(g) (*(gpio+13)&(1<<g))
 
// Adresse de base pour les registres correspondant aux GPIO de la Raspberry Pi :
volatile unsigned* gpio;

 
// Définition de l'adresse de base pour l'utilisation directe des GPIO de la Raspberry Pi :
void setup_gpio_address()
{
	int mem_fd;
	void* gpio_map;

	// Accès à la mémoire physique :
	if ( ( mem_fd = open( "/dev/gpiomem", O_RDWR | O_SYNC ) ) < 0 )
	{
		perror( "Can't open /dev/gpiomem" );
		exit( -1 );
	}

	// Création d'une zone d'adressage vers la mémoire physique correspondant
	// aux registres des GPIO :
	gpio_map = mmap(
		NULL,
		0xB4,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		mem_fd,
		0
	);

	close( mem_fd );

	if ( gpio_map == MAP_FAILED )
	{
		perror( "mmap failed" );
		exit( -1 );
	}

	gpio = (volatile unsigned*) gpio_map;
}

#endif
