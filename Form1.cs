using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace WindowsFormsApp_agv
{
    // 将状态存储到对象中以供UI更新
    public class AgvStatus
    {
        public short X; // 位置X (10mm)
        public short Y; // 位置Y (10mm)
        public short Yaw; // 朝向角 (0.001 rad)
        public short PathPoint; // 当前路径点索引
        public short PathSeg; // 当前路径段索引
        public short Vx; // 速度 (mm/s)
        public short Vy; // 速度 (mm/s)
        public short W; // 角速度 (0.001 rad/s)
        public short Mode; // 0=手动, 1=自动
        public short RunState; // 运行状态 1=停止, 2=启动, 3=暂停
        public short NaviState; // 导航状态 0=空闲, 1=导航中, 2=避障中, 3=导航错误
    }

    public partial class Form1 : Form
    {
        private TcpListener _server;
        private Thread _listenThread;
        private bool _isRunning = false;

        private List<AgvStateControl> _agvControls = new List<AgvStateControl>();
        private DateTime[] _lastUpdateTicks = new DateTime[3];
        private System.Windows.Forms.Timer _timeoutTimer;

        // Modbus Holding Registers (40001+) 模拟内存映射
        // 【寄存器地址分配】
        // AGV #1 状态区: 0-19 | 命令区: 50-51
        // AGV #2 状态区: 100-119 | 命令区: 150-151
        // AGV #3 状态区: 200-219 | 命令区: 250-251
        private ushort[] _holdingRegisters = new ushort[300];

        public Form1()
        {
            InitializeComponent();
            InitCustomUI();
            StartModbusServer();

            // 启动心跳检测定时器
            _timeoutTimer = new System.Windows.Forms.Timer { Interval = 1000 };
            _timeoutTimer.Tick += TimeoutTimer_Tick;
            _timeoutTimer.Start();
        }

        private void InitCustomUI()
        {
            this.Text = "AGV 集群控制调度系统 (Modbus TCP Server)";
            this.Size = new Size(800, 500);
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.ForeColor = Color.White;
            this.Font = new Font("Segoe UI", 10F, FontStyle.Regular, GraphicsUnit.Point, ((byte)(0)));

            TableLayoutPanel mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                Padding = new Padding(20)
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 60));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
            this.Controls.Add(mainLayout);

            // 主标题
            Label lblTitle = new Label
            {
                Text = "AGV 实时监控中心",
                Font = new Font("Segoe UI", 24F, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                ForeColor = Color.FromArgb(0, 122, 204)
            };

            // 右上角logo
            Label lblLogo = new Label
            {
                Text = "宁波视佳舞台",
                Font = new Font("微软雅黑", 16F, FontStyle.Bold),
                AutoSize = true,
                ForeColor = Color.FromArgb(255, 193, 7),
                BackColor = Color.Transparent,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
                TextAlign = ContentAlignment.TopRight,
                Margin = new Padding(0, 0, 10, 0)
            };

            // 用Panel叠加主标题和logo
            Panel titlePanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Transparent
            };
            titlePanel.Controls.Add(lblTitle);
            titlePanel.Controls.Add(lblLogo);
            lblTitle.BringToFront();
            // 设置logo位置（右上角）
            lblLogo.Anchor = AnchorStyles.Top | AnchorStyles.Right;
            int logoMargin = 4;
            lblLogo.Location = new Point(titlePanel.Width - lblLogo.Width - logoMargin, logoMargin);
            lblLogo.BringToFront();
            // 响应Panel大小变化，动态调整logo位置
            titlePanel.Resize += (s, e) =>
            {
                lblLogo.Location = new Point(titlePanel.Width - lblLogo.Width - logoMargin, logoMargin);
            };

            mainLayout.Controls.Add(titlePanel, 0, 0);

            TableLayoutPanel agvLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 3,
                RowCount = 1
            };
            agvLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33F));
            agvLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33F));
            agvLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33F));

            for (int i = 1; i <= 3; i++)
            {
                var agvControl = new AgvStateControl(i);
                agvLayout.Controls.Add(agvControl, i - 1, 0);
                _agvControls.Add(agvControl);
                _lastUpdateTicks[i - 1] = DateTime.MinValue; // 初始化为断开
            }
            mainLayout.Controls.Add(agvLayout, 0, 1);

            // 自适应按钮布局
            TableLayoutPanel buttonLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 7,
                RowCount = 1,
                BackColor = Color.Transparent
            };
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40F)); // 左弹性空白
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));      // 启动
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 40F)); // 间隔
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));      // 停止
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 40F)); // 间隔
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));      // 暂停
            buttonLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40F)); // 右弹性空白

            Button btnStart = CreateStyledButton("全局启动", Color.FromArgb(40, 167, 69));
            btnStart.Margin = new Padding(0);
            btnStart.Click += (s, e) => SendCommandToAll(1);
            Button btnStop = CreateStyledButton("全局停止", Color.FromArgb(220, 53, 69));
            btnStop.Margin = new Padding(0);
            btnStop.Click += (s, e) => SendCommandToAll(2);
            Button btnPause = CreateStyledButton("全局暂停", Color.FromArgb(255, 193, 7));
            btnPause.Margin = new Padding(0);
            btnPause.Click += (s, e) => SendCommandToAll(3);

            buttonLayout.Controls.Add(new Label { AutoSize = true, BackColor = Color.Transparent }, 0, 0); // 左空白
            buttonLayout.Controls.Add(btnStart, 1, 0);
            buttonLayout.Controls.Add(new Label { AutoSize = false, Width = 40, BackColor = Color.Transparent }, 2, 0); // 间隔
            buttonLayout.Controls.Add(btnStop, 3, 0);
            buttonLayout.Controls.Add(new Label { AutoSize = false, Width = 40, BackColor = Color.Transparent }, 4, 0); // 间隔
            buttonLayout.Controls.Add(btnPause, 5, 0);
            buttonLayout.Controls.Add(new Label { AutoSize = true, BackColor = Color.Transparent }, 6, 0); // 右空白

            mainLayout.Controls.Add(buttonLayout, 0, 2);
        }

        private Button CreateStyledButton(string text, Color backColor)
        {
            return new Button
            {
                Text = text,
                Size = new Size(120, 45),
                FlatStyle = FlatStyle.Flat,
                BackColor = backColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 12F, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
        }

        private void StartModbusServer()
        {
            _server = new TcpListener(IPAddress.Any, 502); // 默认Modbus TCP端口
            _isRunning = true;
            _server.Start();
            _listenThread = new Thread(ListenForClients);
            _listenThread.IsBackground = true;
            _listenThread.Start();
        }

        private void ListenForClients()
        {
            while (_isRunning)
            {
                try
                {
                    TcpClient client = _server.AcceptTcpClient();
                    Thread clientThread = new Thread(HandleModbusClient);
                    clientThread.IsBackground = true;
                    clientThread.Start(client);
                }
                catch { }
            }
        }

        private void HandleModbusClient(object obj)
        {
            TcpClient client = (TcpClient)obj;
            NetworkStream stream = client.GetStream();
            byte[] header = new byte[7];

            try
            {
                while (_isRunning)
                {
                    if (!ReadExact(stream, header, 7)) break;

                    int len = (header[4] << 8) | header[5];
                    byte[] pdu = new byte[len - 1]; // 去除UnitID的剩余部分
                    if (!ReadExact(stream, pdu, len - 1)) break;

                    byte fc = pdu[0];

                    if (fc == 3) // Read Holding Registers (PLC读取PC下发的命令)
                    {
                        int startAddr = (pdu[1] << 8) | pdu[2];
                        int qty = (pdu[3] << 8) | pdu[4];

                        byte[] resp = new byte[7 + 2 + qty * 2];
                        Array.Copy(header, resp, 7);
                        int respLen = 3 + qty * 2;
                        resp[4] = (byte)(respLen >> 8);
                        resp[5] = (byte)(respLen & 0xFF);
                        resp[7] = 3;
                        resp[8] = (byte)(qty * 2);

                        for (int i = 0; i < qty; i++)
                        {
                            int addr = startAddr + i;
                            ushort val = addr < _holdingRegisters.Length ? _holdingRegisters[addr] : (ushort)0;
                            resp[9 + i * 2] = (byte)(val >> 8);
                            resp[10 + i * 2] = (byte)(val & 0xFF);
                        }
                        stream.Write(resp, 0, resp.Length);
                    }
                    else if (fc == 16) // Write Multiple Registers (PLC写状态到PC)
                    {
                        int startAddr = (pdu[1] << 8) | pdu[2];
                        int qty = (pdu[3] << 8) | pdu[4];
                        int bytesCount = pdu[5];

                        for (int i = 0; i < qty; i++)
                        {
                            int addr = startAddr + i;
                            if (addr < _holdingRegisters.Length)
                            {
                                _holdingRegisters[addr] = (ushort)((pdu[6 + i * 2] << 8) | pdu[7 + i * 2]);
                            }
                        }

                        byte[] resp = new byte[12];
                        Array.Copy(header, resp, 7);
                        resp[4] = 0;
                        resp[5] = 6;
                        resp[7] = 16;
                        resp[8] = pdu[1];
                        resp[9] = pdu[2];
                        resp[10] = pdu[3];
                        resp[11] = pdu[4];
                        stream.Write(resp, 0, resp.Length);

                        ProcessReceivedRegisters(startAddr);
                    }
                }
            }
            catch { }
            finally { client.Close(); }
        }

        private bool ReadExact(NetworkStream s, byte[] buf, int len)
        {
            int pos = 0;
            while (pos < len)
            {
                int r = s.Read(buf, pos, len - pos);
                if (r == 0) return false;
                pos += r;
            }
            return true;
        }

        // 解析寄存器数据并映射到UI
        private void ProcessReceivedRegisters(int startAddr)
        {
            int agvIndex = -1;
            if (startAddr >= 0 && startAddr < 50) agvIndex = 0;
            else if (startAddr >= 100 && startAddr < 150) agvIndex = 1;
            else if (startAddr >= 200 && startAddr < 250) agvIndex = 2;

            if (agvIndex >= 0)
            {
                _lastUpdateTicks[agvIndex] = DateTime.Now;

                int b = agvIndex * 100;
                AgvStatus status = new AgvStatus
                {
                    X = (short)_holdingRegisters[b + 0],
                    Y = (short)_holdingRegisters[b + 1],
                    Yaw = (short)_holdingRegisters[b + 2],
                    PathPoint = (short)_holdingRegisters[b + 3],
                    PathSeg = (short)_holdingRegisters[b + 4],
                    Vx = (short)_holdingRegisters[b + 5],
                    Vy = (short)_holdingRegisters[b + 6],
                    W = (short)_holdingRegisters[b + 7],
                    Mode = (short)_holdingRegisters[b + 8],
                    RunState = (short)_holdingRegisters[b + 9],
                    NaviState = (short)_holdingRegisters[b + 10]
                };

                UpdateAgvData(agvIndex + 1, status);
            }
        }

        private void SendCommandToAll(int cmdTypeVal)
        {
            // 向 3 台 AGV 发送命令对应寄存器
            // cmdTypeVal: 1=启, 2=停, 3=暂停
            // 修改对应的命令寄存器
            for (int i = 0; i < 3; i++)
            {
                int cmdBase = i * 100 + 50;
                _holdingRegisters[cmdBase] = (ushort)cmdTypeVal;  // 50 / 150 / 250 -> CmdType
                _holdingRegisters[cmdBase + 1] = 1;              // 51 / 151 / 251 -> CmdValue / Trigger Flag
            }
            string cmdText;
            switch (cmdTypeVal)
            {
                case 1: cmdText = "启动"; break;
                case 2: cmdText = "停止"; break;
                case 3: cmdText = "暂停"; break;
                default: cmdText = $"未知({cmdTypeVal})"; break;
            }
            MessageBox.Show($"已在寄存器下发指令: {cmdText}", "下发成功");
        }

        private void TimeoutTimer_Tick(object sender, EventArgs e)
        {
            DateTime now = DateTime.Now;
            for (int i = 0; i < 3; i++)
            {
                if ((now - _lastUpdateTicks[i]).TotalSeconds > 2)
                {
                    _agvControls[i].SetConnection(false);
                }
            }
        }

        public void UpdateAgvData(int id, AgvStatus status)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action(() => UpdateAgvData(id, status)));
                return;
            }
            if (id >= 1 && id <= 3)
            {
                _agvControls[id - 1].SetConnection(true);
                _agvControls[id - 1].UpdateData(status);
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            _isRunning = false;
            _server?.Stop();
            Environment.Exit(0);
            base.OnFormClosing(e);
        }
    }

    // 自定义AGV状态控件，包含UI和状态标签
    public class AgvStateControl : GroupBox
    {
        private Label lblConnStatus;
        private Label lblPos;
        private Label lblPath;
        private Label lblSpeed;
        private Label lblState;

        public AgvStateControl(int id)
        {
            this.Text = $"AGV #{id}";
            this.ForeColor = Color.White;
            this.Font = new Font("Segoe UI", 12F, FontStyle.Bold);
            this.Dock = DockStyle.Fill;
            this.Margin = new Padding(10);
            this.Padding = new Padding(15);

            this.BackColor = Color.FromArgb(30, 30, 30);

            TableLayoutPanel layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 5
            };
            this.Controls.Add(layout);

            lblConnStatus = CreateLabel("状态: [等待连接...]", Color.Gray);
            lblPos = CreateLabel("位置: X=0.0 Y=0.0 Yaw=0.0", Color.LightGray);
            lblPath = CreateLabel("路径: 点 0 / 段 0", Color.LightGray);
            lblSpeed = CreateLabel("速度: Vx=0.0 Vy=0.0 W=0.0", Color.LightGray);
            lblState = CreateLabel("模式: 停止 (未知)", Color.LightGray);

            layout.Controls.Add(lblConnStatus);
            layout.Controls.Add(lblPos);
            layout.Controls.Add(lblPath);
            layout.Controls.Add(lblSpeed);
            layout.Controls.Add(lblState);
        }

        private Label CreateLabel(string text, Color color)
        {
            return new Label
            {
                Text = text,
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", 11F, FontStyle.Regular),
                ForeColor = color,
                TextAlign = ContentAlignment.MiddleLeft
            };
        }

        public void SetConnection(bool connected)
        {
            if (connected)
            {
                lblConnStatus.Text = $"状态: [在线]";
                lblConnStatus.ForeColor = Color.Lime;
            }
            else
            {
                lblConnStatus.Text = "状态: [离线]";
                lblConnStatus.ForeColor = Color.Red;
            }
        }

        public void UpdateData(AgvStatus status)
        {
            // 单位换算
            double x = status.X / 100.0; // 10mm -> m
            double y = status.Y / 100.0;
            double yawDeg = status.Yaw * 0.001 * 180.0 / Math.PI; // 0.001rad -> deg
            double vx = status.Vx / 1000.0; // mm/s -> m/s
            double vy = status.Vy / 1000.0;
            double wDeg = status.W * 0.001 * 180.0 / Math.PI; // 0.001rad/s -> deg/s

            lblPos.Text = $"位置: X={x:F2}m Y={y:F2}m Yaw={yawDeg:F1}°";
            lblPath.Text = $"路径: 点 {status.PathPoint} / 段 {status.PathSeg}";
            lblSpeed.Text = $"速度: Vf={vx:F3}m/s Vl={vy:F3}m/s Va={wDeg:F1}°/s";

            string[] modeStr = { "手动", "自动", "导航" };
            string[] runStr = { "停止", "启动", "暂停" };
            string mode = status.Mode < modeStr.Length ? modeStr[status.Mode] : $"未知({status.Mode})";
            string run = status.RunState < runStr.Length ? runStr[status.RunState] : $"未知({status.RunState})";
            string[] naviStr = { "空闲", "导航中", "避障中", "导航错误" };
            string navi = status.NaviState < naviStr.Length ? naviStr[status.NaviState] : $"未知({status.NaviState})";
            lblState.Text = $"模式: {mode} ({run}) 导航: {navi}";
        }
    }
}
