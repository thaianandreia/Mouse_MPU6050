using System;
using System.Windows.Forms;
using System.IO.Ports;
using System.Linq;
using System.Collections.Generic;
using System.Threading;

namespace ControlMouse
{
    public partial class Form1 : Form
    {
        protected SerialPort mySerialPort = new SerialPort("COM4");
        protected bool flag_L, flag_R;
        protected string valorX, valorY;

        public Form1()
        {
            //InitializeComponent();

            flag_L = false;
            flag_R = false;

            try
            {
                Thread leitura = new Thread(DataReceivedHandler);
                leitura.Start();
            }
            catch
            {
                Encerrar("Algum erro ocorreu.");
            }
        }

        private void DataReceivedHandler()
        {
            mySerialPort.BaudRate = 9600;
            mySerialPort.Parity = Parity.None;
            mySerialPort.StopBits = StopBits.One;
            mySerialPort.DataBits = 8;
            mySerialPort.Handshake = Handshake.None;
            mySerialPort.RtsEnable = true;

            try
            {
                mySerialPort.Open();
            }
            catch
            {
                Encerrar("Porta incorreta!");
            }

            while (1 == 1)
            {
                try
                {
                    System.Threading.Thread.Sleep(50);
                    teste(mySerialPort.ReadLine());
                }
                catch
                {
                    Encerrar("Algum erro ocorreu.");
                }

            }
        }

        private void teste(string buffer)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<string>(teste), new object[] { buffer });
                return;
            }
            List<string> itens = buffer.Split(';').ToList();

            if (flag_L && !buffer.Contains("segura_esquerda"))
            {
                VirtualMouse.LeftUp();
                flag_L = false;
            }

            if (flag_R && !buffer.Contains("segura_direita"))
            {
                VirtualMouse.RightUp();
                flag_R = false;
            }

            if (buffer.Contains("segura_esquerda"))
            {
                VirtualMouse.LeftDown();
                flag_L = true;
            }

            if (buffer.Contains("segura_direita"))
            {
                VirtualMouse.RightDown();
                flag_R = true;
            }

            if (itens.Count >= 2)
            {
                try {
                    valorX = itens[0];
                    valorY = itens[1];

                    double anguloX = double.Parse(valorX.Replace(".", ","));
                    double anguloY = double.Parse(valorY.Replace(".", ","));

                    anguloX = anguloX > 110 || anguloX < -110 ? 0 : anguloX;

                    double velocidadeX = Math.Cos(anguloX) * 5;
                    double velocidadeY = Math.Cos(anguloY) * 5;


                    int distanciaX = int.Parse(Math.Round((Math.Abs(anguloX / 10)) * anguloX / 15).ToString());
                    int distanciaY = int.Parse(Math.Round((Math.Abs(anguloY / 10)) * anguloY / 15).ToString());

                    VirtualMouse.Move(distanciaX, distanciaY);
                }
                catch
                {
                    //Continue
                }
            }
        }

        private void Encerrar(string mensagem)
        {
            DialogResult result = MessageBox.Show(
                                    mensagem,
                                    "Erro!", MessageBoxButtons.OK);

            if (result == DialogResult.OK)
            {
                mySerialPort.Close();
                Application.Exit();
                Environment.Exit(1);
            }
        }
    }
}
