#define F_CPU 7372800 UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE(((F_CPU / (USART_BAUDRATE * 16 UL))) - 1)

#include <avr/io.h>

#include <util/delay.h>

#include <avr/interrupt.h>

#include <string.h>

#include <ctype.h>

#include "lcd.h"

volatile char REC;
int position;
int reset;
char letters[32];
int letterPosition;
int letterFlag;

void USART_Initialization(unsigned int BAUD) {
  UBRRL = BAUD_PRESCALE;
  UBRRH = (BAUD_PRESCALE >> 8);
  UCSRB |= (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
  UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

void Display_Reset(void) {
  lcd_clrscr();
  lcd_gotoxy(0, 0);
  position = 0;
  reset = 0;
}

void String_Reset(void) {
  memset(letters, 0, 32);
  letters[0] = '\0';
  letterPosition = 0;
  letterFlag = 0;
}

void Greeting() {
  char * pCh;
  pCh = strstr(letters, "karlo");
  if (pCh != NULL) {
    lcd_puts("Hej Karlo");
  } else {
    pCh = strstr(letters, "mauro");
    if (pCh != NULL) {
      lcd_puts("Hej Mauro");
    } else {
      pCh = strstr(letters, "rea");
      if (pCh != NULL) {
        lcd_puts("Hej Rea");
      } else {
        lcd_puts("Ne prepoznajem");
        lcd_gotoxy(0, 1);
        lcd_puts("te");
      }
    }
  }

  lcd_gotoxy(0, 0);
}

ISR(USART_RXC_vect) {
  REC = UDR;
}

ISR(INT0_vect) {
  Display_Reset();
}

ISR(INT1_vect) {
  Display_Reset();
  Greeting();
}

int main(void) {
  DDRD = _BV(4);
  TCCR1A = _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS11);
  OCR1B = 18;
  MCUCR = _BV(ISC01) | _BV(ISC11);
  GICR = _BV(INT0) | _BV(INT1);

  lcd_init(LCD_DISP_ON);
  USART_Initialization(USART_BAUDRATE);
  sei();

  lcd_clrscr();
  char data[2];
  data[0] = 0;
  data[1] = 0;

  position = 0;
  reset = 0;
  letterPosition = 0;
  letterFlag = 0;

  memset(letters, 0, 32);

  while (1) {
    int i = 0;
    if (reset == 1 && REC != 0) {
      Display_Reset();
      String_Reset();
    }

    if (REC != 0) {
      if (REC == 10) {
        reset = 1;
      }
      position++;
      data[i++] = REC - 64;

      if (letterFlag == 0) {
        char tmp = REC - 64;
        if (isalpha(tmp)) {
          letters[letterPosition] = tolower(tmp);
          letterPosition++;
          letters[letterPosition] = '\0';
        }

        if (letterPosition >= 31) {
          letterFlag = 1;
        }
      }

      REC = 0;
    }
    data[i] = '\0';
    if (position > 32) {
      data[0] = '\0';
    }
    lcd_puts(data);
    if (position == 16) {
      lcd_gotoxy(0, 1);
    }
  }
}