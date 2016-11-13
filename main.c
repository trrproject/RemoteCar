#define sei()   __asm__ __volatile__ ("sei" ::)
#define cli()   __asm__ __volatile__ ("cli" ::)
#define trama_max 5
#include <avr/io.h>
#include <avr/interrupt.h>

void USART_Init(uint16_t ubrr_value);
void USARTWriteChar(unsigned char data);
void USART_Transmit( unsigned int data );
unsigned char CRC8x8(unsigned char input);
unsigned char CRC8(unsigned char input, unsigned char seed);

volatile int posicao = 0;
volatile int modo = 0;
int tempo = 0;
volatile unsigned char trama[trama_max];
volatile int soma;

int main(void){
	
	// Inicializacao dos registos
	TCCR1A = 0X00;
	TCCR1B = 0X00;
	TCCR2  = 0X00;
	TCNT0  = 0x00;

	// Activacao do contador responsavel pela inactividade do motor
	TCCR0 |= (1<<CS02) | (0<<CS01) | (1<<CS00);

	// Mascara que permite saber quando atinge o overfolw	
	TIMSK |= (1<<TOIE0);
	// Ativacao das portas para saıda
	DDRB |= (1<<PB3) | (1<<PB1) | (1<<PB0);

	// Activacao dos registos para colocar o PWM em funcionamento
	TCCR1A|= (1<<COM1A1)| (1<<WGM11) | (0<<COM1A0) | (0<<COM1B0) | (0<<COM1B1) |(0<<FOC1A) | (0<<FOC1B) | (0<<WGM10);
	TCCR1B|= (1<<WGM13) | (1<<WGM12) | (1<<CS11)   | (0<<CS12)   | (0<<ICNC1)  |(0<<ICES1) | (0<<CS10);
	TCCR2|=  (0<<WGM21) | (1<<WGM20) | (1<<CS22)   | (1<<CS21)   |(1<<CS20)  | (1<<COM21) |   (0<<COM20);

	ICR1=20000;	
	OCR1A = 1465;
	OCR2 = 0;

  	sei();
	USART_Init(51);
	
	while(1){
		//Tratamento da trama
		if (modo == 2) {
			USARTWriteChar('S');
			soma = trama[0] + trama[1] + trama[2] + trama[3];
				if (trama[4] == CRC8x8(soma)){
					USARTWriteChar('N');
					if (trama[0] == '#'){
							if (trama[1] == 'S'){
								OCR1A = (255*trama[2])+trama[3];
							}
							else if (trama[1] == 'F'){
								OCR2 = ((0*trama[2])+trama[3]);
								PORTB=(0<<PB0);
							}
							else if (trama[1] == 'T'){
								PORTB=(1<<PB0);
								OCR2 = ((0*trama[2])+trama[3]);
							}
					}
				}
			// Coloca o temporizador de inactividade a zero
			modo = 0;
			TCNT0 = 0x00;
			tempo = 0;
		}
	}
	return 1;
	
	
}

// Temporizador de inactividade ( interrupcao )
SIGNAL(TIMER0_OVF_vect) {
	tempo += 1;
	if (tempo == 7){
		TCNT0 = 0x00;
		OCR2= 0;
		tempo = 0;
	}
}

// Interrupcao capaz de detectar actividade de recepcao (USART)
ISR(USART_RXC_vect){
	//Tratamento e preenchimento da trama
	uint8_t temp;
	temp = UDR;
	
	if (modo == 1){
		trama[posicao] = temp;
		posicao++;
	}

	if (modo  == 0){
		if (temp == '#'){
			posicao = 0;
			trama[posicao] = temp;
			modo = 1;
			posicao++;
		}
	}
	
	if (posicao == trama_max){
		modo = 2;
		posicao = 0;
	}
}

// Inicializacao o da comunicacao de serie
void USART_Init(uint16_t ubrr_value){
	UBRRL = ubrr_value;
    UBRRH = (ubrr_value>>8);
	UCSRB = (1<<RXEN)|(1<<TXEN);
	UCSRB|= (1<<RXCIE);
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
}

//Funcao de transmissao com numeros inteiros
void USART_Transmit( unsigned int data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	/* Copy ninth bit to TXB8 */
	UCSRB &= ~(1<<TXB8);
	if ( data & 0x0100 )
	UCSRB |= (1<<TXB8);
	/* Put data into buffer, sends the data */
	UDR = data;
}

void USARTWriteChar(unsigned char data){
	while ( !( UCSRA & (1<<UDRE)) );
		UDR = data;	
}

// Funcoes responsaveis pelo CRC
unsigned char CRC8(unsigned char input, unsigned char seed)
{
    unsigned char i, feedback;
    for (i=0; i<8; i++)
    {
        feedback = ((seed ^ input) & 0x01);
        if (!feedback) seed >>= 1;
        else
        {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }
        input >>= 1;
    }
    return seed;    
} 

unsigned char CRC8x8(unsigned char input)
{
    unsigned char i, check;

    check=0;
    for (i=0; i<8; i++)
    {
        check = CRC8(input, check);
        input++;
    }
    return check;
} 