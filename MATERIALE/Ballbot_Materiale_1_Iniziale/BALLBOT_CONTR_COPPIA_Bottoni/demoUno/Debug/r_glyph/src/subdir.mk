################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../r_glyph/src/r_glyph.c \
../r_glyph/src/r_glyph_register.c 

COMPILER_OBJS += \
r_glyph/src/r_glyph.obj \
r_glyph/src/r_glyph_register.obj 

# Each subdirectory must supply rules for building sources it contributes
r_glyph/src/%.obj: ../r_glyph/src/%.c 
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	ccrx -MM -MP -output=dep="$(@:%.obj=%.d)"  -MT="$(@:%.d=%.obj)"  -MT="$(@:%.obj=%.d)" -cpu=rx600 -fpu -include="C:\Workspace\demoUno\r_bsp","C:\Workspace\demoUno\r_glyph","C:\Workspace\demoUno\r_glyph\src","C:\Workspace\demoUno\r_glyph\src\glyph","C:\Workspace\demoUno\r_rspi_rx600","C:\Workspace\demoUno\r_rspi_rx600\src","C:\Workspace\demoUno\src","C:\Program Files (x86)\Renesas\RX\2_7_0\include" -define=__RX -lang=c99 -nomessage -debug -optimize=0 -nologo -nologo  "$<"
	ccrx -cpu=rx600 -fpu -include="C:\Workspace\demoUno\r_bsp","C:\Workspace\demoUno\r_glyph","C:\Workspace\demoUno\r_glyph\src","C:\Workspace\demoUno\r_glyph\src\glyph","C:\Workspace\demoUno\r_rspi_rx600","C:\Workspace\demoUno\r_rspi_rx600\src","C:\Workspace\demoUno\src","C:\Program Files (x86)\Renesas\RX\2_7_0\include" -define=__RX -lang=c99 -nomessage -output=obj -obj_path="r_glyph/src" -debug -optimize=0 -nologo -nologo "$<"
	@echo 'Finished Scanning and building: $<'
	@echo.
