###########################################
#Makefile for simple programs
###########################################
INC=
LIB= -lpthread

CC=gcc
CC_FLAG=-Wall

PRG=test_new
OBJ=test_new.o

$(PRG):$(OBJ)
	$(CC) $(INC)  -o $@ $(OBJ) $(LIB)
	
.SUFFIXES: .c .o .cpp
.cpp.o:
	$(CC) $(CC_FLAG) $(INC) -c $*.cpp -o $*.o

.PRONY:clean
clean:
	@echo "Removing linked and compiled files......"
	rm -f $(OBJ) $(PRG)
