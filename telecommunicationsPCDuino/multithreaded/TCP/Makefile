DIR=$(shell pwd)

INCS = -I.\
        -I$(DIR) \
        -I$(DIR)/c_environment/hardware \
        -I$(DIR)/c_environment/hardware/arduino \
        -I$(DIR)/c_environment/hardware/arduino/cores \
        -I$(DIR)/c_environment/hardware/arduino/cores/arduino \
        -I$(DIR)/c_environment/hardware/arduino/variants \
        -I$(DIR)/c_environment/hardware/arduino/variants/sunxi \
        -I$(DIR)/c_environment/libraries \
        -I$(DIR)/c_environment/libraries/Serial \
        -I$(DIR)/c_environment/libraries/SPI \
	-I$(DIR)/c_environment/libraries/Wire \
	-I$(DIR)/c_environment/libraries/LiquidCrystal \
	-I$(DIR)/c_environment/libraries/PN532_SPI \
	-pthread

LIBS=$(DIR)/c_environment/libarduino.a
TARGET=$(DIR)

OBJS = brain
all: 
	@mkdir -p $(TARGET)
	@for i in $(OBJS); do echo "$(CXX) $(INCS) $$i.c -o  $(TARGET)/$$i $(LIBS)" && $(CXX) $(INCS) $$i.c -o $(TARGET)/$$i $(LIBS); done

clean:
	@for i in $(OBJS); do rm -f $(TARGET)/$$i; done
