using System;
using System.Device.I2c;
using System.Diagnostics;
using System.Numerics;
using Iot.Device.Adxl345;

namespace Iot.Device.ADXL354_I2C
{
    /// <summary>
    /// I2C Accelerometer ADXL354_I2C
    /// </summary>
    public class ADXL354_I2C : IDisposable
    {

        /// <summary>
        /// The default I2C address of ADXL354_I2C device
        /// </summary>
        public const byte DefaultI2CAddress = 0x53;
        private I2cDevice _i2CDevice;
        private GravityRange AccelerometerRange = GravityRange.Range16;

        
        enum Register : byte
        {
            /*
            Each I2C slave device needs to have a unique I2C address. 
            The ADXL345 has an ALT address pin that can be hardwired to set the I2C address of this digital sensor. 
            
            If the ALT ADDRESS pin is pulled HIGH in a module, the 7-bit I2C address for the device is 0x1D — followed by the R/W bit. 
            This translates to 0x3A for a write and 0x3B for a read. 
            
            If the ALT ADDRESS pin is connected to the ground, the 7-bit I2C address for the device is 0x53 (followed by the R/W bit). 
            This translates to 0xA6 for a write and 0xA7 for a read. 
            
            In a module, the ALT ADDRESS pin is already pulled HIGH or LOW. 
            The I2C address of the ADXL345 sensor used in this tutorial is 0x53. 
            */

            ACCEL_I2C_ADDR = 0x53,           /* 7-bit I2C address of the ADXL345 with SDO pulled low */
            ACCEL_REG_POWER_CONTROL = 0x2D,  /* Address of the Power Control register */
            ACCEL_REG_DATA_FORMAT = 0x31,    /* Address of the Data Format register   */
            ACCEL_REG_X = 0x32,              /* Address of the X Axis data register   */
            ACCEL_REG_Y = 0x34,              /* Address of the Y Axis data register   */
            ACCEL_REG_Z = 0x36              /* Address of the Z Axis data register   */
        }
       

      
        /// <summary>
        /// Constructs a ADXL354 I2C device.
        /// </summary>
        /// <param name="i2CDevice">The I2C device used for communication.</param>
        /// <param name="accelerometerRange">The sensitivity of the accelerometer.</param>
        public ADXL354_I2C(I2cDevice i2CDevice, GravityRange accelerometerRange = GravityRange.Range16)
        {
            _i2CDevice = i2CDevice ?? throw new ArgumentNullException(nameof(i2CDevice));
            AccelerometerRange = accelerometerRange;

        }

        public void SetupDefault()
        {
            
            WriteRegister ( Register.ACCEL_REG_DATA_FORMAT, (byte)AccelerometerRange);       /* 0x01 sets range to +- 16Gs                         */
            WriteRegister ( Register.ACCEL_REG_POWER_CONTROL, 0x08 );                       /* 0x08 puts the accelerometer into measurement mode */
        }

        public void IsDevicePresent()
        {
           
            byte result = _i2CDevice.ReadByte();

            byte[] array = new byte[1];

            I2cTransferResult txfrResult = _i2CDevice.Read(array);
            
            switch ( txfrResult.Status)
            {
                case I2cTransferStatus.UnknownError:
                    Debug.WriteLine("I2cTransferStatus.UnknownError");
                    ////     The transfer failed for an unknown reason.
                    break;           
                case I2cTransferStatus.FullTransfer:
                    Debug.WriteLine("I2cTransferStatus.FullTransfer");
                    //// Summary:
                    ////     The data was entirely transferred. For WriteRead, the data for both the write
                    ////     and the read operations was entirely transferred. For this status code, the value
                    ////     of the System.Device.I2c.I2cTransferResult.BytesTransferred member that the method
                    ////     returns is the same as the size of the buffer you specified when you called the
                    ////     method, or is equal to the sum of the sizes of two buffers that you specified
                    ////     for WriteRead.
                    break;
                case I2cTransferStatus.PartialTransfer:
                    Debug.WriteLine("I2cTransferStatus.PartialTransfer");
                    /// //// Summary:
                    ////     The I2C device negatively acknowledged the data transfer before all of the data
                    ////     was transferred. For this status code, the value of the System.Device.I2c.I2cTransferResult.BytesTransferred
                    ////     member that the method returns is the number of bytes actually transferred. For
                    ////     System.Device.I2c.I2cDevice.WriteRead(System.SpanByte,System.SpanByte), the value
                    ////     is the sum of the number of bytes that the operation wrote and the number of
                    ////     bytes that the operation read.
                    break;
                case I2cTransferStatus.SlaveAddressNotAcknowledged:
                    Debug.WriteLine("I2cTransferStatus.SlaveAddressNotAcknowledged");
                    //// Summary:
                    ////     The bus address was not acknowledged. For this status code, the value of the
                    ////     System.Device.I2c.I2cTransferResult.BytesTransferred member that the method returns
                    ////     of the method is 0.
                    break;
                case I2cTransferStatus.ClockStretchTimeout:
                    Debug.WriteLine("I2cTransferStatus.ClockStretchTimeout:");
                    //// Summary:
                    ////     The transfer failed due to the clock being stretched for too long. Ensure the
                    ////     clock line is not being held low.
                    break;
            }

            if (txfrResult.BytesTransferred >= 1)
            {
                //Should equal address ACCEL_I2C_ADDR
                int i = 0;
                foreach (byte reply in array)
                {
                    Debug.WriteLine($"Data[{i}] = {reply}");
                    i++;
                }
            }
            else
            {
                Debug.WriteLine($"No Device present");
            }
        }
      

        public Vector3 ReadI2CAccel()
        {
            int ACCEL_DYN_RANGE_G = 4;
            const int ACCEL_RES = 1024;       //The ADXL345 has 10 bit resolution giving 1024 unique values
            switch(AccelerometerRange)
            {
                case GravityRange.Range02:
                    ACCEL_DYN_RANGE_G = 4;
                    break;
                case GravityRange.Range04:
                    ACCEL_DYN_RANGE_G = 8;
                    break;
                case GravityRange.Range08:
                    ACCEL_DYN_RANGE_G = 16;
                    break;
                case GravityRange.Range16:
                    ACCEL_DYN_RANGE_G = 32;
                    break;
            }
            
            int UNITS_PER_G = ACCEL_RES / ACCEL_DYN_RANGE_G;  /* Ratio of raw int values to G units                          */

            byte[] RegAddrBuf = new byte[] { (byte) Register.ACCEL_REG_X }; /* Register address we want to read from                                         */
            byte[] ReadBuf = new byte[6];                   /* We read 6 bytes sequentially to get all 3 two-byte axes registers in one read */

            /* 
             * Read from the accelerometer 
             * We call WriteRead() so we first write the address of the X-Axis I2C register, then read all 3 axes
             */
            _i2CDevice.WriteRead(RegAddrBuf, ReadBuf);

            /* 
             * In order to get the raw 16-bit data values, we need to concatenate two 8-bit bytes from the I2C read for each axis.
             * We accomplish this by using the BitConverter class.
             */
            short AccelerationRawX = BitConverter.ToInt16(ReadBuf, 0);
            short AccelerationRawY = BitConverter.ToInt16(ReadBuf, 2);
            short AccelerationRawZ = BitConverter.ToInt16(ReadBuf, 4);

            /* Convert raw values to G's */
            Vector3 accel;
            accel.X = (double)AccelerationRawX / UNITS_PER_G;
            accel.Y = (double)AccelerationRawY / UNITS_PER_G;
            accel.Z = (double)AccelerationRawZ / UNITS_PER_G;

            return accel;
        }

        private void WriteRegister(Register register, byte data)
        {
            SpanByte dataout = new byte[]
            {
                (byte)register, data
            };

            _i2CDevice.Write(dataout);
        }

        private byte ReadByte(Register register)
        {
            _i2CDevice.WriteByte((byte)register);
            return _i2CDevice.ReadByte();
        }

        private void ReadBytes(Register register, SpanByte readBytes)
        {
            _i2CDevice.WriteByte((byte)register);
            _i2CDevice.Read(readBytes);
        }

        /// <inheritdoc />
        public void Dispose()
        {
            _i2CDevice.Dispose();
            _i2CDevice = null!;
        }
    }
}