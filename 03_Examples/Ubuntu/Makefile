main:imu_data_decode.o packet.o main.o
	gcc -o main $^ -lpthread

%.o:%.c
	gcc -c -o $@ $<
	
.PHONY:clean
clean:
	rm *.o main

