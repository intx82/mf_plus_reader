using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using LibUsbDotNet;
using LibUsbDotNet.Main;
using System.Text;
using System.Threading.Tasks;

namespace mfp_initializer
{
    class Program
    {
        /// <summary>
        /// Таймаут по умолчанию, в мс
        /// </summary>
        public const int DEFAULT_OPERATION_TIMEOUT = 2000;

        /// <summary>
        /// Размер данных передаваемых между EP по-умолчанию
        /// </summary>
        public const int DEFAULT_EP_DATA_SIZE = 128;

        /// <summary>
        /// PID устройства
        /// </summary>
        public const int MIFARE_READER_PID = 0x16c0;
        /// <summary>
        /// VID устройства
        /// </summary>
        public const int MIFARE_READER_VID = 0xc001;
        /// <summary>
        /// Класс устройства
        /// </summary>
        public static UsbDevice mifare_reader_dev;
        /// <summary>
        /// Класс для поиска устройства на шине
        /// </summary>
        public static UsbDeviceFinder mifare_reader_finder = new UsbDeviceFinder(MIFARE_READER_PID, MIFARE_READER_VID);



        /// <summary>
        /// Устанавливает Команду устройства
        /// </summary>
        /// <param name="cmd">Команда устройству.</param>
        /// <see cref="mifare_reader_cmd_resp_t"/>
        /// <returns>true - если все хорошо</returns>
        static bool set_cmd(mifare_reader_cmd_resp_t cmd)
        {
            UsbSetupPacket set_state_packet = new UsbSetupPacket((byte)(UsbCtrlFlags.Direction_Out | UsbCtrlFlags.Recipient_Interface | UsbCtrlFlags.RequestType_Vendor), 0, 2, 0, 0);
            int out_len = 0;
            return mifare_reader_dev.ControlTransfer(ref set_state_packet, cmd.serialize_cmd(), mifare_reader_cmd_resp_t.len, out out_len);
        }

        /// <summary>
        /// Получает состояние выполнения комманды
        /// </summary>
        /// <param name="state">Состояние устройства</param>
        /// <returns>True - если состояние цифрового автомата в устройстве получено</returns>
        static bool get_state(out mifare_reader_cmd_resp_t state)
        {
            byte[] o = new byte[2];
            UsbSetupPacket set_state_packet = new UsbSetupPacket((byte)(UsbCtrlFlags.Direction_In | UsbCtrlFlags.Recipient_Interface | UsbCtrlFlags.RequestType_Vendor), 0, 2, 0, 0);
            int out_len = 0;
            bool res = mifare_reader_dev.ControlTransfer(ref set_state_packet, o, mifare_reader_cmd_resp_t.len, out out_len);
            state = new mifare_reader_cmd_resp_t(o);
            return res;
        }


        /// <summary>
        /// Записывает данные в устройство (В READER_DATA_IN_EP)
        /// </summary>
        /// <param name="data">Данные для записи, должны быть размером 32 байта, иначе будет плохо</param>
        /// <returns>Количество отправленных байт</returns>
        static ErrorCode write_to_dev(byte[] data, out int out_len)
        {
            UsbEndpointWriter writer = mifare_reader_dev.OpenEndpointWriter(WriteEndpointID.Ep01, EndpointType.Bulk);
            return writer.Write(data, DEFAULT_OPERATION_TIMEOUT, out out_len);
        }

        /// <summary>
        /// Читает данные с устройства (В READER_DATA_OUT_EP)
        /// </summary>
        /// <param name="data">Считанные данные</param>
        /// <returns>Количество принятых байт</returns>
        static ErrorCode read_from_dev(byte[] data, out int out_len)
        {
            UsbEndpointReader reader = mifare_reader_dev.OpenEndpointReader(ReadEndpointID.Ep01, DEFAULT_EP_DATA_SIZE, EndpointType.Bulk);
            return reader.Read(data, DEFAULT_OPERATION_TIMEOUT, out out_len);
        }

        /// <summary>
        /// Отправляет команду устройству
        /// </summary>
        /// <param name="cmd">Команда</param>
        /// <param name="data_in">Данные для отправки в устройство</param>
        /// <param name="data_out">Данные принятые от устройтсва</param>
        /// <returns></returns>
        static mifare_reader_cmd_resp_t do_cmd(mifare_reader_cmd_resp_t.num_t cmd, byte[] data_in, out byte[] data_out)
        {
            data_out = new Byte[128];
            mifare_reader_cmd_resp_t resp = new mifare_reader_cmd_resp_t(mifare_reader_cmd_resp_t.num_t.IDLE);
            resp.resp = (mifare_reader_cmd_resp_t.resp_t)0xff;
            if (data_in.Length > 0)
            {
                if (data_in.Length > DEFAULT_EP_DATA_SIZE)
                {
                    return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
                }
                int out_len = 0;
                byte[] b = new byte[128];
                Array.Copy(data_in, b, data_in.Length);
                if (write_to_dev(data_in, out out_len) != ErrorCode.Ok)
                {
                    return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
                }

            }
            if (!set_cmd(new mifare_reader_cmd_resp_t(cmd, (byte)data_in.Length)))
            {
                return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
            }
            int cnt = 0xff;

            do
            {
                Thread.Sleep(10);
                if (!get_state(out resp))
                {
                    return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
                }
                if (cnt > 0)
                {
                    cnt--;
                }
                else
                {
                    return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
                }
            } while (((int)resp.resp & 0x80) != 0x80);
            Thread.Sleep(1);
            if (resp.resp == mifare_reader_cmd_resp_t.resp_t.OK)
            {
                int out_len = 0;
                if (resp.data_size > 0)
                {
                    if (read_from_dev(data_out, out out_len) != ErrorCode.Ok)
                    {
                        return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
                    }
                    Array.Resize(ref data_out, out_len);
                }
            }
            return resp;
        }
        /// <summary>
        /// Отправляет команду REQA_WUPA 
        /// Если карта присутствует то ответ OK, если нет, но считыватель ответил - Timeout, иначе 0xff
        /// </summary>
        /// <param name="sak">SAK карты (2 байта)</param>
        /// <returns>Класс команды/ответа</returns>
        static mifare_reader_cmd_resp_t reqa_wupa_cmd(out byte[] sak)
        {
            sak = new Byte[0];
            return do_cmd(mifare_reader_cmd_resp_t.num_t.REQA_WUPA, new byte[0], out sak);
        }

        /// <summary>
        /// Отправляет команду SELECT 
        /// Если карта присутствует то ответ OK, если нет, но считыватель ответил - Timeout, иначе 0xff
        /// </summary>
        /// <param name="serial">Серийный номер карты 12 байт (uid)</param>
        /// <returns>Класс команды/ответа</returns>
        static mifare_reader_cmd_resp_t select_cmd(out byte[] serial)
        {
            serial = new Byte[0];
            return do_cmd(mifare_reader_cmd_resp_t.num_t.SELECT, new byte[0], out serial);
        }

        /// <summary>
        /// Отправляет команду COMM 
        /// Если карта присутствует то ответ OK, если нет, но считыватель ответил - Timeout, иначе 0xff
        /// </summary>
        /// <param name="data_in">Входные данные, которые будут отправленны карте</param>
        /// <param name="data_out">Выходные данные, которые были приняты с карты</param>
        /// <returns>Класс команды/ответа</returns>
        static mifare_reader_cmd_resp_t comm_cmd(byte[] data_in, out byte[] data_out)
        {
            data_out = new Byte[0];
            return do_cmd(mifare_reader_cmd_resp_t.num_t.COMM, data_in, out data_out);
        }

        /// <summary>
        /// Останавливает передачу данных в формате CRYPTO1
        /// Если карта присутствует то ответ OK, если нет, но считыватель ответил - Timeout, иначе 0xff
        /// </summary>
        /// <returns>Класс команды/ответа</returns>
        static mifare_reader_cmd_resp_t stop_crypto_cmd()
        {
            byte[] data_out = new Byte[0];
            return do_cmd(mifare_reader_cmd_resp_t.num_t.STOP_CRYPTO1, new Byte[0], out data_out);
        }
        /// <summary>
        /// Авторизируется на карте
        /// </summary>
        /// <param name="block">Номер блока</param>
        /// <param name="key">Ключ для авторизации</param>
        /// <param name="serial">Серийный номер карты из SELECT-а</param>
        /// <returns>Класс команды/ответа</returns>
        static mifare_reader_cmd_resp_t mf_auth_cmd(int block, byte[] key, byte[] serial)
        {
            byte[] data_out = new Byte[0];
            byte[] data_in = new byte[19]; /**< 1+6+(1+10+1) */
            if ((key.Length < 6) || (serial.Length < 12))
            {
                return new mifare_reader_cmd_resp_t((mifare_reader_cmd_resp_t.num_t)0xff);
            }
            data_in[0] = (byte)block;
            Array.Copy(key, 0, data_in, 1, 6);
            Array.Copy(serial, 0, data_in, 7, 12);

            return do_cmd(mifare_reader_cmd_resp_t.num_t.MF_AUTH, data_in, out data_out);
        }

        static mifare_reader_cmd_resp_t read_dev_serial(out byte[] serial)
        {
            serial = new Byte[0];
            return do_cmd(mifare_reader_cmd_resp_t.num_t.READER_SERIAL, new byte[0], out serial);
        }



        /// <summary>
        /// Открывает и иниациализирует устройство
        /// </summary>
        /// <returns>True - если устройство найдено и работает</returns>
        static bool mifare_reader_open_dev()
        {
            mifare_reader_dev = UsbDevice.OpenUsbDevice(mifare_reader_finder);
            if (mifare_reader_dev == null)
            {
                return false;
            }

            IUsbDevice wholeUsbDevice = mifare_reader_dev as IUsbDevice;
            if (!ReferenceEquals(wholeUsbDevice, null))
            {
                wholeUsbDevice.SetConfiguration(1);
                wholeUsbDevice.ClaimInterface(0);
            }
            return true;
        }
        static void Main(string[] args)
        {
            Console.CancelKeyPress += delegate
            {
                Console.WriteLine("Для выхода нажмите любую клавишу");
                Console.ReadKey();
                Environment.Exit(1);
            };

            try
            {
                if (!mifare_reader_open_dev())
                {
                    Console.WriteLine("{0,0}{1,20}", "Поиск устройства:", "Устройство не найдено");
                    throw new Exception();
                }
                Console.WriteLine("{0,0}{1,20}", "Поиск устройства:", "Устройство найдено");
                byte[] dev_serial;
                mifare_reader_cmd_resp_t r = read_dev_serial(out dev_serial);
                if (r.resp == mifare_reader_cmd_resp_t.resp_t.OK)
                {
                    Console.WriteLine("{0,0}{1,20}", "Серийный номер считывателя: ", BitConverter.ToString(dev_serial).Replace("-", string.Empty));
                }
                else
                {
                    Console.WriteLine("{0,0}{1,20}", "Серийный номер считывателя: ", "Ошибка");
                    throw new Exception();
                }
                Console.WriteLine("{0,0}", "Поднесите карту к считывателю");
                while (true)
                {
                    
                    byte[] sak;
                    r = reqa_wupa_cmd(out sak);
                    if (r.resp == mifare_reader_cmd_resp_t.resp_t.OK)
                    {
                        Console.Clear();
                        Console.WriteLine("{0,0}{1,20}", "REQA: ", BitConverter.ToString(sak).Replace("-", string.Empty));
                        byte[] serial;
                        Thread.Sleep(10);
                        r = select_cmd(out serial);
                        if (r.resp == mifare_reader_cmd_resp_t.resp_t.OK)
                        {   
                            Console.WriteLine("{0,0}{1,20}", "SELECT: ", BitConverter.ToString(serial,1, serial[0]).Replace("-", ":"));
                            byte[] cmd = new byte[2] { 0xe0,0x50 };
                            Console.WriteLine("COMM: > {0}",  BitConverter.ToString(cmd).Replace("-", ":"));
                            Console.WriteLine("COMM: [{0}] < {1}", comm_cmd(cmd, out cmd).resp.ToString("G"), BitConverter.ToString(cmd).Replace("-", ":"));
                            cmd = new byte[2] { 0x50, 0 };
                            Console.WriteLine("COMM: > {0}",  BitConverter.ToString(cmd).Replace("-", ":"));
                            Console.WriteLine("COMM: [{0}] < {1}", comm_cmd(cmd, out cmd).resp.ToString("G"), BitConverter.ToString(cmd).Replace("-", ":"));

                            Thread.Sleep(500);
                        }
                    }
                }

            }
            catch
            {

            }
            Console.WriteLine("Для выхода нажмите любую клавишу");
            Console.ReadKey();
        }
    }

    #region Клас команды считывателю
    class mifare_reader_cmd_resp_t
    {
        /// <summary>
        /// Размер команды по-умолчанию
        /// </summary>
        public const int len = 2;
        /// <summary>
        /// Комманды считывателя
        /// </summary>
        public enum num_t : short
        {
            /// <summary>
            /// Без команды
            /// </summary>
            IDLE = 0,
            /// <summary>
            /// Отправляет карте REQA-WUPA команду, по ответу можно узнать есть ли карта в поле. Если успешно возвращает тип карты (SAK)
            /// Размер возвращаемых данных – 2 байта
            /// </summary>
            REQA_WUPA = 1,
            /// <summary>
            /// Выбирает карту, возвращает структуру с данными о карте
            /// Struct {
            /// uint8_t data_size;
            /// uint8_t uidByte[10];
            /// uint8_t sak
            /// }; 
            /// Размер возвращаемых данных 12 байт
            /// </summary>
            SELECT = 2,
            ///<summary>
            /// Отправляет команду карте, указанную в теле команды
            ///</summary>
            COMM = 3,
            /// <summary>
            /// Выключает шифрование CRYPTO1, Должно быть обязательно выполнено после завершения работы с MF-Classic картой.
            /// </summary>
            STOP_CRYPTO1 = 4,
            /// <summary>
            /// Аутентифицируется на MF-Classic карте 
            /// Формат запроса: 1 байта номера блока + 6 байт кода доступа к блоку + 12 байт номера карты (В формате полученном с SELECT-а)
            /// </summary>
            MF_AUTH = 5,
            /// <summary>
            /// Отдает серийный номер считывателя 12 байт
            /// </summary>
            READER_SERIAL = 6,
        };

        /// <summary>
        /// Возможные ответы от устройства
        /// </summary>
        public enum resp_t
        {
            /// <summary>
            /// Команда выполнена успешно
            /// </summary>
            OK = 0x80,
            /// <summary>
            /// Error in communication
            /// </summary>
            COMM_ERROR,
            /// <summary>
            /// Collission detected
            /// </summary>
            COLLISION_ERROR,
            /// <summary>
            /// Timeout in communication.
            /// </summary>
            TIMEOUT,
            /// <summary>
            ///  A buffer is not big enough.
            /// </summary>
            NO_ROOM,
            /// <summary>
            /// Internal error in the code. Should not happen ;-)
            /// </summary>
            INTERNAL_ERROR,
            /// <summary>
            ///  Invalid argument.
            /// </summary>
            INVALID_ARG,
            /// <summary>
            ///  The CRC_A does not match
            /// </summary>
            STATUS_CRC_WRONG,
        };

        /// <summary>
        /// Номер команды
        /// <see cref="num_t"/>
        /// </summary>
        public num_t num;
        /// <summary>
        /// Возможные ответы устройства
        /// </summary>
        public resp_t resp;
        /// <summary>
        /// Размер прилагаемых данных к команде
        /// </summary>
        public byte data_size;

        /// <summary>
        /// Конструктор принимающий Номер команды и размер данных
        /// </summary>
        /// <param name="n">Номер команды</param>
        /// <param name="s">Размер данных</param>
        public mifare_reader_cmd_resp_t(num_t n, byte s)
        {
            resp = (resp_t)0xff;
            data_size = s;
            num = n;
        }
        /// <summary>
        /// Конструктор без указания размера
        /// </summary>
        /// <param name="n">Номер комманды</param>
        public mifare_reader_cmd_resp_t(num_t n)
        {
            resp = (resp_t)0xff;
            num = n;
            data_size = 0;
        }
        /// <summary>
        /// Конструктор десериализатор
        /// </summary>
        /// <param name="r">Данные принятые с ридера</param>
        public mifare_reader_cmd_resp_t(byte[] r)
        {
            if (r.Length != 2)
            {
                return;
            }

            if ((r[0] & 0x80) == 0x80)
            {
                resp = (resp_t)r[0];
            }
            else
            {
                num = (num_t)r[0];
            }
            data_size = r[1];
        }
        /// <summary>
        /// Сериализирует данные класа в массив байт
        /// </summary>
        /// <returns>Массив байт с данными класса</returns>
        public byte[] serialize_cmd()
        {
            return new byte[] { (byte)num, data_size };
        }
        #endregion

    }

}
