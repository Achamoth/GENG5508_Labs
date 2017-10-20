# Makefile T. Braunl 2017, Compile all .c into .x
ALL:  $(patsubst %.c,%.x, $(wildcard *.c))
	
%.x: %.c
	../gccsim -g -o $*.x $*.c
	
clean:
	$(RM) *.x
