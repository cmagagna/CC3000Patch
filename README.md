<h1>The Arduino CC3000 Firmware Upgrader</h1>
<h2>Version 0.1 - September 19, 2013 - Chris Magagna</h2>
<h3>This will upgrade a CC3000 to:
		Service Pack Version 1.24
		Release Package 1.11.1
</h3>

<p>					
	This code released into the public domain, with no usefulness implied blah
	blah blah.
</p>
	
<pre>
!!!!WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING
!	
!	WARNING! This will delete the current firmware on your CC3000. If this
!	upgrade fails you will be left with a useless piece of metal where
!	your nice WiFi module used to be.
!	
!!!!WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING-WARNING
</pre>

<p>	
	Having said that, I've tested this code now on 2 CC3000 modules I made
	myself, an AdaFruit CC3000 breakout
	(http://www.adafruit.com/products/1469), and an official TI CC3000EM
	(https://estore.ti.com/CC3000EM-CC3000-Evaluation-Module-P4257.aspx)
	and so far so good.
</p>

<p>	
	This code has been tested with an Arduino Nano upgraded to the OptiBoot
	bootloader and a Uno R3. The code takes up nearly the entire 32K Flash
	space of an ATMega328 so it won't fit on a board with a larger
	bootloader, e.g. the Duemilanove, stock Nano, etc.
</p>

<p>
	This version does NOT work with the Teensy 3. I'm working on it!
</p>

<p>
	Many thanks to AdaFruit for the Adafruit_CC3000 library.
</p>

<p>	
	This code is based on a combination of the TI patch programmers:
	PatchProgrammerMSP430F5529_6_11_7_14_24windows_installer.exe
	PatchProgrammerMSP430FR5739_1_11_7_14_24windows_installer.exe
	Both are available on the T.I. website.
</p>
	
<p>
	To use:
</p>

<ol>	
	<li>Quit the Arduino IDE, if running</li>
	<li>Move the folder "Adafruit_CC3000_Library" out of your Library. This code
		uses a custom version of their library, and the two can't coexist. You
		won't be able to compile / upload this program if Arduino sees them both.</li>
	<li>Edit the pin assignments CC3000_IRQ, CC3000_VBAT, and CC3000_CS for your
		setup, as necessary.</li>
	<li>Wire up your CC3000 to your Arduino</li>
	<li>Upload this sketch to your Arduino</li>
	<li>Open the Serial Monitor with baud rate 115200</li>
	<li>Use option 0 to verify everything is working (see notes below)</li>
	<li>Use option 4Y to back up your CC3000's info to your Arduino's EEPROM</li>
	<li>Use option 6Y erase the CC3000's current firmware</li>
	<li>Use option 7Y to restore your CC3000's info from Arduino EEPROM</li>
	<li>Use option 8Y to update the first part of the new firmware</li>
	<li>Use option 9Y to update the second part of the new firmware</li>
	<li>Use option 0 to verify the CC3000 is now working with your new firmware</li>
</ol>

<p>
	For options 0 - 3 you just type in the number and hit Enter or press 'Send'
	but options 4-9 require you to also type UPPERCASE Y or D to confirm you
	want to make a (potentially) destructive change.
</p>

<p>	
	If the CC3000 initializes but you can't read the MAC or the firmware
	version then everything else may be working but the CC3000's NVRAM may be
	corrupt. You may be able to recover from this by:
</p>

<ol>	
	<li>Restart your Arduino</li>
	<li>Use option 5D to generate a new random MAC address and load the T.I.
		defaults</li>
	<li>Use option 6Y erase the CC3000's current firmware</li>
	<li>Use option 7Y to restore your CC3000's info from Arduino EEPROM</li>
	<li>Use option 8Y to update the first part of the new firmware</li>
	<li>Use option 9Y to update the second part of the new firmware</li>
	<li>Use option 0 to verify the CC3000 is now working with your new firmware</li>
</ol>