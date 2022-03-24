
//Restore nuget packages without update > Update-Package -reinstall


// Install firmware downloader application nanoff (Nanoframework Firmware Flasher) 
// --------------
// a. Install Dotnet Core 6
// b. Open developer command prompt and update Nano Firmware Flasher by command
// > dotnet tool update -g nanoff

// if firmware updater installation errors -> remove old v1.x.x had a different name [nanofirmwareflasher] and it conflicts with new name
// > dotnet tool uninstall nanofirmwareflasher  --global

// Execute cmd to ESP32 Nano firmware (change comport to match)
// > nanoff --update --platform esp32 --preview --serialport COM7 -v diag

#region v1.7.4-p.114_ firmware

// Sometimes it is tricky to match the embedded Firmware native assembly with nuget packages
// https://docs.nanoframework.net/content/getting-started-guides/guide-version-checksums.html

// The firmware version is independent from the assembly versions,
// but the latest firmware always uses the latest native assembly versions and will work with the latest NuGet package versions of a component

// 1.7.4 - preview.114
//Native Assemblies:
//  mscorlib v100.5.0.17, checksum 0x004CF1CE
//  nanoFramework.Runtime.Native v100.0.9.0, checksum 0x109F6F22
//  nanoFramework.Hardware.Esp32 v100.0.7.3, checksum 0xBE7FF253
//  nanoFramework.Hardware.Esp32.Rmt v100.0.3.0, checksum 0x9A53BB44
//  nanoFramework.Device.OneWire v100.0.4.0, checksum 0xB95C43B4
//  nanoFramework.Networking.Sntp v100.0.4.4, checksum 0xE2D9BDED
//  nanoFramework.ResourceManager v100.0.0.1, checksum 0xDCD7DF4D
//  nanoFramework.System.Collections v100.0.1.0, checksum 0x2DC2B090
//  nanoFramework.System.Text v100.0.0.1, checksum 0x8E6EB73D
//  nanoFramework.Runtime.Events v100.0.8.0, checksum 0x0EAB00C9
//  EventSink v1.0.0.0, checksum 0xF32F4C3E
//  System.IO.FileSystem v1.0.0.0, checksum 0x3AB74021
//  System.Math v100.0.5.4, checksum 0x46092CB1
//  System.Net v100.1.4.1, checksum 0xA01012C3
//  Windows.Devices.Adc v100.1.3.3, checksum 0xCA03579A
//  System.Device.Adc v100.0.0.0, checksum 0xE5B80F0B
//  System.Device.Dac v100.0.0.6, checksum 0x02B3E860
//  System.Device.Gpio v100.1.0.4, checksum 0xB6D0ACC1
//  Windows.Devices.Gpio v100.1.2.2, checksum 0xC41539BE
//  Windows.Devices.I2c v100.2.0.2, checksum 0x79EDBF71
//  System.Device.I2c v100.0.0.1, checksum 0xFA806D33
//  Windows.Devices.Pwm v100.1.3.3, checksum 0xBA2E2251
//  System.Device.Pwm v100.1.0.4, checksum 0xABF532C3
//  Windows.Devices.SerialCommunication v100.1.1.2, checksum 0x34BAF06E
//  System.IO.Ports v100.1.4.0, checksum 0xCB7C0ECA
//  Windows.Devices.Spi v100.1.4.2, checksum 0x360239F1
//  System.Device.Spi v100.1.1.0, checksum 0x3F6E2A7E
//  System.Device.WiFi v100.0.6.3, checksum 0x5C9E06C4
//  Windows.Storage v100.0.2.0, checksum 0x954A4192

#endregion


using System;
using System.Diagnostics;
using System.Threading;
using nanoFramework.Hardware.Esp32;
using System.Device.Gpio;
using System.Device.Spi;
using System.Numerics;
using Iot.Device.Adxl345;
using System.Device.I2c;
using Iot.Device.ADXL354_I2C;


namespace ScanWiFi
{
    public class Program
    {
        
        public static void Main()
        {
            Debug.WriteLine($"\r\n\r\n\r\n\r\n\r\n\r\n--------- Startup {DateTime.UtcNow.AddHours(8).ToString("hh:mm tt")} ---------------------");
           
            //ADXL345_Spi_Test(); //Wont work?

            //SWitched to i2c 
            Adxl345_i2c_Test();

            
           
        }


        static void Adxl345_i2c_Test()
        {
            
            //////////////////////////////////////////////////////////////////////
            // when connecting to an ESP32 device,
            // configure the I2C GPIOs used for the bus
            Configuration.SetPinFunction(21, DeviceFunction.I2C1_DATA);
            Configuration.SetPinFunction(22, DeviceFunction.I2C1_CLOCK);


            I2cConnectionSettings i2CConnectionSettings = new I2cConnectionSettings(1, ADXL354_I2C.DefaultI2CAddress);
            I2cDevice device = I2cDevice.Create(i2CConnectionSettings);

            using ADXL354_I2C sensor = new ADXL354_I2C(device, GravityRange.Range16);

            int i = 0;
            while (true)
            {
                // read data
                Vector3 data = sensor.ReadI2CAccel();
                Debug.WriteLine($"#{i}");
                Debug.WriteLine($"X: {data.X.ToString("0.00")} g");
                Debug.WriteLine($"Y: {data.Y.ToString("0.00")} g");
                Debug.WriteLine($"Z: {data.Z.ToString("0.00")} g");
                Debug.WriteLine("\r\n");
                // wait for 500ms
                Thread.Sleep(500);
            }


        }        
        static void ADXL345_Spi_Test()
        {
           
            // ESP32 Devkit C - argh!!! SPI Wont work - switching to I2C

                                // ESP32     Function      ADXL345         
                                // ---------------------------------
            int MOSI = 23;      // GPIO 23 - SPI1 MOSI -     SDA
            int MISO = 19;      // GPIO 19 - SPI1 MISO -     SDO 
            int SPI_CLK = 18;   // GPIO 18 - SPI1 SCLK -     CLK 
            int SPI_CS = 5;     // GPIO  5 - SPI1 CS   -     CS

            Configuration.SetPinFunction(MOSI, DeviceFunction.SPI1_MOSI);       // SDA  
            Configuration.SetPinFunction(MISO, DeviceFunction.SPI1_MISO);       // SCL  
            Configuration.SetPinFunction(SPI_CLK, DeviceFunction.SPI1_CLOCK);   // 23

            SpiConnectionSettings settings = new SpiConnectionSettings(1, SPI_CS) //IO5 = CS line
            {
                ClockFrequency = Adxl345.SpiClockFrequency,
                Mode = Adxl345.SpiMode,
                ChipSelectLineActiveState = PinValue.Low
            };

            Debug.WriteLine(">System.Device.Spi Init ");
            SpiDevice spiDevice = SpiDevice.Create(settings);
            SpiConnectionSettings connectionSettings;
            Debug.WriteLine("->SpiDevice.GetBusInfo(1)");
            // You can get the values of SpiBus
            SpiBusInfo spiBusInfo = SpiDevice.GetBusInfo(1);
          //  Debug.WriteLine($"{nameof(spiBusInfo.ChipSelectLineCount)}: {spiBusInfo.ChipSelectLineCount}");
            Debug.WriteLine($"{nameof(spiBusInfo.MaxClockFrequency)}: {spiBusInfo.MaxClockFrequency}");
            Debug.WriteLine($"{nameof(spiBusInfo.MinClockFrequency)}: {spiBusInfo.MinClockFrequency}");
           // Debug.WriteLine($"{nameof(spiBusInfo.SupportedDataBitLengths)}: ");
            //foreach (var data in spiBusInfo.SupportedDataBitLengths)
            //{
            //    Debug.WriteLine($"  {data}");

            //}

            SpiConnectionSettings spiConnectionSettings = spiDevice.ConnectionSettings;
            Debug.WriteLine("\r\n->spiDevice.ConnectionSettings");
            Debug.WriteLine($"{nameof(spiConnectionSettings.BusId)}: {spiConnectionSettings.BusId}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.ChipSelectLine)}: {spiConnectionSettings.ChipSelectLine}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.ChipSelectLineActiveState)}: {spiConnectionSettings.ChipSelectLineActiveState}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.ClockFrequency)}: {spiConnectionSettings.ClockFrequency}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.DataBitLength)}: {spiConnectionSettings.DataBitLength}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.DataFlow)}: {spiConnectionSettings.DataFlow}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.Mode)}: {spiConnectionSettings.Mode}");
            Debug.WriteLine($"{nameof(spiConnectionSettings.SharingMode)}: {spiConnectionSettings.SharingMode}");
            
            Debug.WriteLine("\r\nSPI init.. ok");
            
            //https://github.com/nanoframework/nanoFramework.IoT.Device/tree/develop/devices/Adxl345
            // Set gravity measurement range ±4G
            Adxl345 MotionSensor = new Adxl345(spiDevice, GravityRange.Range04);
            Debug.WriteLine("Adxl345 Device...");

            if (MotionSensor == null)
            {
                Debug.WriteLine("Adxl345 == null");
            }

            Debug.WriteLine("Adxl345 Motion Testing....");
            int i = 0;
            // Loop forever
            while (true)
            {
                Vector3 data = MotionSensor.Acceleration;
                Debug.WriteLine($"#{i}");
                Debug.WriteLine($"X: {data.X.ToString("0.00")} g");
                Debug.WriteLine($"Y: {data.Y.ToString("0.00")} g");
                Debug.WriteLine($"Z: {data.Z.ToString("0.00")} g\r\n");
                Thread.Sleep(500);  // 5 seconds 
            }

            return;


            // -----------------------------
            // Debugging Low level SPI work 
            // ----------------------------
            // Note: the ChipSelect pin should be adjusted to your device, 
            connectionSettings = new SpiConnectionSettings(1, SPI_CS);
            // You can adjust other settings as well in the connection
            connectionSettings.ClockFrequency = 1_000_000;
            connectionSettings.DataBitLength = 8;
            connectionSettings.DataFlow = DataFlow.LsbFirst;
            connectionSettings.Mode = SpiMode.Mode2;

            // Then you create your SPI device by passing your settings
            spiDevice = SpiDevice.Create(connectionSettings);

            // You can write a SpanByte
            SpanByte writeBufferSpanByte = new byte[2] { 42, 84 };
            spiDevice.Write(writeBufferSpanByte);
            // Or a ushort buffer
            ushort[] writeBufferushort = new ushort[2] { 4200, 8432 };
            spiDevice.Write(writeBufferushort);
            // Or simply a byte
            spiDevice.WriteByte(42);

            // The read operations are similar
            SpanByte readBufferSpanByte = new byte[2];
            // This will read 2 bytes
            spiDevice.Read(readBufferSpanByte);
            ushort[] readUshort = new ushort[4];
            // This will read 4 ushort
            spiDevice.Read(readUshort);
            // read 1 byte
            byte readMe = spiDevice.ReadByte();
            Debug.WriteLine($"I just read a byte {readMe}");

            // And you can operate full transferts as well
            SpanByte writeBuffer = new byte[4] { 0xAA, 0xBB, 0xCC, 0x42 };
            SpanByte readBuffer = new byte[4];
            spiDevice.TransferFullDuplex(writeBuffer, readBuffer);
            // Same for ushirt arrays:
            ushort[] writeBufferus = new ushort[4] { 0xAABC, 0x00BB, 0xCC00, 0x4242 };
            ushort[] readBufferus = new ushort[4];
            spiDevice.TransferFullDuplex(writeBufferus, readBufferus);

        }

    }
}
