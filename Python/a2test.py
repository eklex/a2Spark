#!/usr/bin/env python3

import a2spark as a2
import time
import sys

def printResult(title, success):
	if success:
		print("== %s: PASS =="%title)
	else:
		print("== %s: FAIL =="%title)

print("\n#################################");
print("#################################");
print("###   a2Spark Routine Test    ###");
print("#################################");
print("#################################\n");

print("Find a2spark device connected...")
deviceAddr = a2.find()

try:
	for i, i2cAddr in enumerate(deviceAddr):
		print("a2Spark #%d found at 0x%02x" % (i,i2cAddr))
except:
	print("!!! No device found. !!!\n!!! Test aborted. !!!")
	sys.exit(1)

for i2cAddr in deviceAddr:
	print("\n\n#################################");
	print("###         a2park @%02x        ###"%i2cAddr);
	print("#################################");
	print("Device address at 0x%02x" % a2.getAddress(i2cAddr))
	print("Firmware version", a2.getFirmware(i2cAddr))

	for i in [1,2]:
		print("\n#################################");
		print("###       Analog%d Test        ###"%i);
		print("#################################");
		success = True
		for cycle in range(3):
			AnEn = a2.getAnalogEn(i, i2cAddr)
			if AnEn:
				print(" -> Analog%d enable"%i)
			else:
				print(" -> Analog%d disable"%i)
			
			LpEn = a2.getLPEn(i, i2cAddr)
			if LpEn:
				print(" -> Low-pass%d enable"%i)
			else:
				print(" -> Low-pass%d disable"%i)
			
			if cycle == 0:
				print("## Toggle bits ##")
			elif cycle == 1:
				print("## Restore bits ##")
			
			if a2.setAnalogEn(i, not AnEn, i2cAddr) != 0:
				print(" !!! setAnalogEn failed !!!")
				success = False
			if a2.setLPEn(i, not LpEn, i2cAddr) != 0:
				print(" !!! setLPEn failed !!!")
				success = False
		printResult("Analog Config test",success)
		
		print("## LP%d Config ##"%i)
		success = True
		# save LP reg value
		LpVal = a2.getLPFilter(i, i2cAddr)
		# test every valid value
		for cycle in range(16):
			if a2.setLPFilter(i, cycle, i2cAddr) != 0:
				print("!!! setLPFilter failed at cycle %d !!!"% cycle)
				print("!!! setLPFilter test aborted !!!")
				success = False
				break
		# restore LP reg value
		a2.setLPFilter(i, LpVal, i2cAddr)
		printResult("LP Config test",success)
		
		print("## Analog%d Sampling ##"%i)
		success = True
		if a2.setAnalogEn(i, True, i2cAddr) != 0:
			print(" !!! setAnalogEn failed !!!")
			success = False
		if a2.setLPEn(i, True, i2cAddr) != 0:
			print(" !!! setLPEn failed !!!")
			success = False
		for cycle in range(10):
			AnVal = a2.getAnalog(i, i2cAddr)
			print("Sample #%02d: Analog%d value=%d"%(cycle,i,AnVal))
			time.sleep(5)
		if a2.setAnalogEn(i, AnEn, i2cAddr) != 0:
			print(" !!! setAnalogEn failed !!!")
			success = False
		if a2.setLPEn(i, LpEn, i2cAddr) != 0:
			print(" !!! setLPEn failed !!!")
			success = False
		printResult("Sampling Analog test",success)
		
		print("\n#################################");
		print("###        Alarm%d Test        ###"%i);
		print("#################################");
		success = True
		for cycle in range(3):
			AlEn = a2.getAlarmEn(i, i2cAddr)
			if AlEn:
				print(" -> Alarm%d enable"%i)
			else:
				print(" -> Alarm%d disable"%i)
			
			AlMoEn = a2.getAlarmModeEn(i, i2cAddr)
			if AlMoEn:
				print(" -> Alarm%d mode enable"%i)
			else:
				print(" -> Alarm%d mode disable"%i)
			
			if cycle == 0:
				if a2.getAlarmOutput(i, i2cAddr):
					print(" -> Alarm%d output hight"%i)
				else:
					print(" -> Alarm%d output low"%i)
			
			if cycle == 0:
				print("## Toggle bits ##")
			elif cycle == 1:
				print("## Restore bits ##")
			
			if a2.setAlarmEn(i, not AlEn, i2cAddr) != 0:
				print("!!! setAlarmEn failed !!!")
				success = False
			if a2.setAlarmModeEn(i, not AlMoEn, i2cAddr) != 0:
				print("!!! setAlarmModeEn failed !!!")
				success = False
		printResult("Alarm Config test",success)
		
		print("## Alarm%d Mode ##"%i)
		success = True
		# save Al Mode reg value
		AlMd = a2.getAlarmMode(i, i2cAddr)
		# test every valid value
		for cycle in range(8):
			if a2.setAlarmMode(i, cycle, i2cAddr) != 0:
				print("!!! setAlarmMode failed at cycle %d !!!"%cycle)
				print("!!! setAlarmMode test aborted !!!")
				success = False
				break
		# restore Al Mode reg value
		a2.setAlarmMode(i, AlMd, i2cAddr)
		printResult("Alarm Mode test",success)
		
		print("## Set & Clear Value Al%d ##"%i)
		success = True
		for cycle in range(2):
			SetVal = a2.getSetValue(i, i2cAddr)
			print(" -> Alarm%d set value=%d"%(i,SetVal))
			ClrVal = a2.getClearValue(i, i2cAddr)
			print(" -> Alarm%d clear value=%d"%(i,ClrVal))
			
			if cycle == 0:
				print("## Registers changed ##")
				if SetVal > 0 and SetVal < 1023/2:
					SetVal += 1023/3
				elif SetVal >= 1023/2:
					SetVal -= 1023/3
				else:
					SetVal = 1023/4
				if a2.setSetValue(i, int(SetVal), i2cAddr) != 0:
					print("!!! setSetValue failed !!!")
					success = False
				else:
					print(" -> Expected set value", SetVal)
				
				if ClrVal > 0 and ClrVal < 1023/2:
					ClrVal += 1023/3
				elif ClrVal >= 1023/2:
					ClrVal -= 1023/3
				else:
					ClrVal = 1023/2
				if a2.setClearValue(i, int(ClrVal), i2cAddr) != 0:
					print("!!! setClearValue failed !!!")
					success = False
				else:
					print(" -> Expected clear value", ClrVal)
		printResult("Set & Clear Value test",success)
		
	print("\n#################################");
	print("###     Temperature Test      ###");
	print("#################################");
	success = True
	for cycle in range(3):
		TpEn = a2.getTempEn(i2cAddr)
		if TpEn:
			print(" -> Temperature enable")
		else:
			print(" -> Temperature disable")
		
		TpOffSng = a2.getTempOffSign(i2cAddr)
		if TpOffSng:
			print(" -> Temperature offset negative")
		else:
			print(" -> Temperature offset positive")
		
		if cycle == 0:
			print("## Toggle bits ##")
		elif cycle == 1:
			print("## Restore bits ##")
		
		if a2.setTempEn(not TpEn, i2cAddr) != 0:
			print("!!! setTempEn failed !!!")
			success = False
		if a2.setTempOffSign(not TpOffSng, i2cAddr) != 0:
			print("!!! setTempOffSign failed !!!")
			success = False
	printResult("Temperature Config test",success)
	
	print("## Temperature Offset Value ##")
	success = True
	# save Temp Offset reg value
	TpOff = a2.getTempOff(i2cAddr)
	for cycle in range(16):
		if a2.setTempOff(cycle, i2cAddr) != 0:
			print("!!! setTempOff failed at cycle %d !!!"%cycle)
			print("!!! setTempOff test aborted !!!")
			success = False
			break
	# restore Temp Offset reg value
	a2.setTempOff(TpOff, i2cAddr)
	printResult("Temperature Offset test",success)
	
	print("## Temperature Unit ##")
	success = True
	# save temp unit
	TpUnit = a2.getTempUnit(i2cAddr)
	for cycle in ["C", "K", "F", "c", "k", "f"]:
		if a2.setTempUnit(cycle, i2cAddr) != 0:
			print("!!! setTempUnit failed at cycle %d !!!"%cycle)
			print("!!! setTempUnit test aborted !!!")
			success = False
			break
	#restore temp unit
	a2.setTempUnit(TpUnit, i2cAddr)
	printResult("Temperature Unit test",success)
	
	print("## Temperature Sampling ##")
	success = True
	if a2.setTempEn(True, i2cAddr) != 0:
		print("!!! setTempEn failed !!!")
		success = False
	for cycle in range(10):
		TpVal = a2.getTemp(i2cAddr)
		print("Sample #%02d: Temperature value=%d%s"%(cycle,TpVal,TpUnit))
		time.sleep(5)
	if a2.setTempEn(TpEn, i2cAddr) != 0:
		print("!!! setTempEn failed !!!")
		success = False
	printResult("Temperature Sampling test",success)

