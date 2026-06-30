using System; // 基础命名空间
using System.Collections.Generic; // 泛型集合
using System.Deployment.Application; // 发布版本相关
using System.Drawing; // 图形和颜色相关
using System.Net; // 网络相关
using System.Net.Sockets; // Socket 通信
using System.Threading; // 多线程
using System.Threading.Tasks; // 异步任务
using System.Windows.Forms; // WinForms UI

namespace WindowsFormsApp_agv
{
    /// <summary>
    /// AGV状态数据结构，包含位置、速度、模式等信息
    /// </summary>
    public class AgvStatus
    {
        public short X;         // X坐标（单位：厘米）
        public short Y;         // Y坐标（单位：厘米）
        public short Yaw;       // 朝向角（单位：0.1°/bit）
        public short PathPoint; // 当前路径点编号
        public short PathSeg;   // 当前路径段编号
        public short Vx;        // 前进速度（单位：mm/s）
        public short Vy;        // 侧向速度（保留）
        public short W;         // 角速度（单位：0.0573°/s）
        public short Mode;      // 工作模式（0手动/1自动/2导航）
        public short RunState;  // 运行状态（0停止/1运行/2暂停）
        public short NaviState; // 导航状态（自定义）
    }

    /// <summary>
    /// 工艺路线中的单个点位信息
    /// </summary>
    public class ProcessPoint
    {
        public short PathId;      // 路径ID
        public short DelayTicks;  // 延时时间（单位：0.1s）
        public short RotateAngle; // 旋转角度（单位：度）
        public short RotateSpeed; // 旋转速度（单位：deg/s）
    }

    /// <summary>
    /// 工艺路线，包含编号和点位列表
    /// </summary>
    public class ProcessRoute
    {
        public int RouteNumber; // 路线编号
        public List<ProcessPoint> Points = new List<ProcessPoint>(); // 路线点位集合
    }


    /// <summary>
    /// 主窗体，负责AGV集群调度系统的UI和核心逻辑
    /// </summary>
    public partial class Form1 : Form
    {
        // AGV数量、路线数量、每条路线点数等参数
        private const int AgvCount = 3;                // AGV数量
        private const int RouteCount = 20;             // 每台AGV的路线数量
        private const int PointsPerRoute = 10;         // 每条路线的点数
        private const int RegistersPerPoint = 4;       // 每个点占用寄存器数
        private const int RegistersPerRoute = PointsPerRoute * RegistersPerPoint; // 每条路线占用寄存器数
        private const int RegistersPerAgv = 1000;      // 每台AGV分配的寄存器空间

        // Modbus寄存器地址偏移量常量
        private const int StatusOffset = 0;            // 状态区起始偏移
        private const int StatusRegisterCount = 20;    // 状态区寄存器数量
        private const int CommandOffset = 50;          // 指令区偏移
        private const int CommandValueOffset = 51;     // 指令值偏移
        private const int SelectedRouteOffset = 52;    // 当前选中路线号偏移
        private const int RouteDataOffset = 100;       // 路线数据区偏移

        // 统一UI颜色配置
        private static readonly Color ThemeBg = Color.FromArgb(36, 39, 46);         // 主背景色
        private static readonly Color PanelBg = Color.FromArgb(28, 30, 36);         // 面板背景色
        private static readonly Color HighlightBlue = Color.FromArgb(78, 163, 255); // 高亮蓝
        private static readonly Color SuccessGreen = Color.FromArgb(33, 150, 83);   // 成功绿
        private static readonly Color DangerRed = Color.FromArgb(210, 72, 72);      // 危险红
        private static readonly Color WarningOrange = Color.FromArgb(224, 168, 55); // 警告橙

        // Modbus服务器相关字段
        private TcpListener _server;           // TCP监听器
        private Thread _listenThread;          // 监听线程
        private bool _isRunning;               // 服务器运行标志

        // UI控件与数据结构
        private readonly List<AgvStateControl> _agvControls = new List<AgvStateControl>(); // AGV状态控件集合
        private readonly List<RouteSelectionControl> _routeControls = new List<RouteSelectionControl>(); // 路线选择控件集合
        private readonly List<List<ProcessRoute>> _agvRoutes = new List<List<ProcessRoute>>(); // 所有AGV的路线库
        private readonly DateTime[] _lastUpdateTicks = new DateTime[AgvCount]; // AGV最后更新时间戳
        private readonly bool[] _isAgvOnline = new bool[AgvCount]; // AGV在线状态记录
        private System.Windows.Forms.Timer _timeoutTimer; // 超时检测定时器

        private readonly ushort[] _holdingRegisters = new ushort[RegistersPerAgv * AgvCount + 100]; // Modbus寄存器区
        private readonly bool[] _holdingCoils = new bool[2000]; // Modbus线圈区 (2000位)

        /// <summary>
        /// 构造函数，初始化UI、数据和服务器
        /// </summary>
        public Form1()
        {
            // 启用双缓冲以解决重绘重影问题，提升界面流畅度
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer |
                           ControlStyles.AllPaintingInWmPaint |
                           ControlStyles.UserPaint, true);
            this.UpdateStyles();

            InitializeComponent(); // 设计器生成UI
            InitializeRouteLibraries(); // 初始化工艺路线库
            InitCustomUI(); // 初始化自定义UI布局
            SyncAllRouteDataToRegisters(); // 路线数据同步到寄存器
            StartModbusServer(); // 启动Modbus服务器

            // 启动超时检测定时器
            _timeoutTimer = new System.Windows.Forms.Timer { Interval = 1000 };
            _timeoutTimer.Tick += TimeoutTimer_Tick;
            _timeoutTimer.Start();
        }

        /// <summary>
        /// 初始化所有AGV的工艺路线库，默认每台AGV有20条路线，每条路线10个点
        /// </summary>
        private void InitializeRouteLibraries()
        {
            _agvRoutes.Clear();
            for (int agvIndex = 0; agvIndex < AgvCount; agvIndex++)
            {
                var routes = new List<ProcessRoute>();
                for (int routeIndex = 0; routeIndex < RouteCount; routeIndex++)
                {
                    var route = new ProcessRoute { RouteNumber = routeIndex + 1 };
                    for (int pointIndex = 0; pointIndex < PointsPerRoute; pointIndex++)
                    {
                        // 默认每个点速度800，其他为0
                        route.Points.Add(new ProcessPoint { PathId = 0, DelayTicks = 0, RotateAngle = 0, RotateSpeed = 10 }); // 默认旋转速度10°/s
                    }
                    routes.Add(route);
                }

                // 尝试从持久化文件加载
                string persistFile = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, $"agv_routes_{agvIndex + 1}.json");
                if (System.IO.File.Exists(persistFile))
                {
                    try
                    {
                        var json = System.IO.File.ReadAllText(persistFile);
                        var loaded = Newtonsoft.Json.JsonConvert.DeserializeObject<List<ProcessRoute>>(json);
                        if (loaded != null && loaded.Count == routes.Count)
                        {
                            for (int i = 0; i < routes.Count; i++)
                            {
                                routes[i].RouteNumber = loaded[i].RouteNumber;
                                for (int j = 0; j < routes[i].Points.Count && j < loaded[i].Points.Count; j++)
                                {
                                    routes[i].Points[j] = loaded[i].Points[j];
                                }
                            }
                        }
                    }
                    catch { /* ignore file errors */ }
                }

                _agvRoutes.Add(routes);
            }
        }

        /// <summary>
        /// 初始化自定义UI布局，包括标题、监控区、路线区和底部按钮
        /// </summary>
        private void InitCustomUI()
        {
            Text = "AGV 集群控制调度系统 (Modbus TCP Server)";
            Size = new Size(900, 500);
            MinimumSize = new Size(600, 300);
            BackColor = ThemeBg;
            ForeColor = Color.White;
            Font = new Font("Segoe UI", 10F);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                Padding = new Padding(12),
                BackColor = ThemeBg
            };

            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 60)); // 标题栏
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 60F)); // 监控区
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 40F)); // 路线区
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 60)); // 底部按钮区
            Controls.Add(mainLayout);

            // 1. 标题区
            var titlePanel = new Panel { Dock = DockStyle.Fill, BackColor = PanelBg, Padding = new Padding(12, 12, 12, 10) };
            var lblTitle = new Label { Text = "AGV 实时监控中心", Font = new Font("Segoe UI", 16F, FontStyle.Bold), Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter, ForeColor = HighlightBlue };

            string version = "1.0.0.0";
            if (ApplicationDeployment.IsNetworkDeployed)
            {
                version = ApplicationDeployment.CurrentDeployment.CurrentVersion.ToString();
            }
            else
            {
                version = Application.ProductVersion;
            }

            var lblLogo = new Label { Text = "Ver: " + version, Font = new Font("微软雅黑", 12F, FontStyle.Bold), AutoSize = true, ForeColor = Color.FromArgb(255, 200, 87) };

            titlePanel.Controls.Add(lblLogo); // 先添加 Logo 确保层级
            titlePanel.Controls.Add(lblTitle);

            titlePanel.Resize += (s, e) =>
            {
                lblLogo.Location = new Point(titlePanel.Width - lblLogo.Width - 10, (titlePanel.Height - lblLogo.Height) / 2);
                lblLogo.BringToFront();
            };
            mainLayout.Controls.Add(titlePanel, 0, 0);

            // 2. 核心显示区 (监控 + 路线)
            string[] names = { "小船", "石头", "松树" };
            mainLayout.Controls.Add(CreateTripleControlGrid(_agvControls, i => new AgvStateControl(i + 1, i < names.Length ? names[i] : "")), 0, 1);
            mainLayout.Controls.Add(CreateTripleControlGrid(_routeControls, i =>
            {
                var ctrl = new RouteSelectionControl(i + 1, RouteCount, i < names.Length ? names[i] : "");
                ctrl.RouteSelectionChanged += (s, e) => SyncSelectedRouteRegister(i);
                ctrl.EditClicked += (s, e) => OpenRouteEditor(i);
                return ctrl;
            }), 0, 2);

            // 3. 按钮区
            var buttonLayout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
                WrapContents = false
            };

            var btnStart = CreateStyledButton("全局启动", SuccessGreen);
            btnStart.Click += (s, e) =>
            {
                if (MessageBox.Show("确定要【启动】所有 AGV 吗？", "操作确认", MessageBoxButtons.YesNo, MessageBoxIcon.Question) == DialogResult.Yes)
                {
                    SendCommandToAll(1);
                }
            };

            var btnStop = CreateStyledButton("全局停止", DangerRed);
            btnStop.Click += (s, e) => SendCommandToAll(2);
            var btnPause = CreateStyledButton("全局暂停", WarningOrange);
            btnPause.Click += (s, e) => SendCommandToAll(3);

            buttonLayout.Controls.AddRange(new Control[] { btnStart, btnStop, btnPause });

            foreach (Control c in buttonLayout.Controls)
                c.Margin = new Padding(20, 0, 20, 0);

            buttonLayout.Layout += (s, e) =>
            {
                int totalWidth = 0;
                foreach (Control c in buttonLayout.Controls) totalWidth += c.Width + c.Margin.Left + c.Margin.Right;
                int leftPadding = (buttonLayout.Width - totalWidth) / 2;
                int topPadding = (buttonLayout.Height - 46) / 2;
                buttonLayout.Padding = new Padding(Math.Max(0, leftPadding), Math.Max(0, topPadding), 0, 0);
            };

            mainLayout.Controls.Add(buttonLayout, 0, 3);
        }

        /// <summary>
        /// 创建三列控件网格（如AGV状态区、路线区）
        /// </summary>
        /// <typeparam name="T">控件类型</typeparam>
        /// <param name="list">控件集合</param>
        /// <param name="factory">控件工厂方法</param>
        /// <returns>TableLayoutPanel</returns>
        private TableLayoutPanel CreateTripleControlGrid<T>(List<T> list, Func<int, T> factory) where T : Control
        {
            var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = AgvCount, RowCount = 1 };
            for (int i = 0; i < AgvCount; i++)
            {
                grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F / AgvCount));
                var ctrl = factory(i);
                list.Add(ctrl);
                grid.Controls.Add(ctrl, i, 0);
            }
            return grid;
        }

        /// <summary>
        /// 创建带样式的按钮
        /// </summary>
        private Button CreateStyledButton(string text, Color backColor)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(140, 46),
                FlatStyle = FlatStyle.Flat,
                BackColor = backColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 12F, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }

        /// <summary>
        /// 打开指定AGV的路线编辑器窗口
        /// </summary>
        /// <param name="agvIndex">AGV索引</param>
        private void OpenRouteEditor(int agvIndex)
        {
            string[] names = { "小船", "石头", "松树" };
            string name = agvIndex < names.Length ? names[agvIndex] : "";
            using (var editor = new RouteEditorForm(agvIndex + 1, _agvRoutes[agvIndex], name, delegate
            {
                SyncRouteLibraryToRegisters(agvIndex);
            }))
            {
                editor.ShowDialog(this);
            }
        }

        /// <summary>
        /// 获取指定AGV的寄存器基址
        /// </summary>
        private int GetAgvBase(int agvIdx) => agvIdx * RegistersPerAgv;

        /// <summary>
        /// 将指定AGV的路线库同步到Modbus寄存器区
        /// </summary>
        private void SyncRouteLibraryToRegisters(int agvIdx)
        {
            var routes = _agvRoutes[agvIdx];
            int baseAddr = GetAgvBase(agvIdx) + RouteDataOffset;
            for (int r = 0; r < RouteCount; r++)
            {
                for (int p = 0; p < PointsPerRoute; p++)
                {
                    int addr = baseAddr + (r * RegistersPerRoute) + (p * RegistersPerPoint);
                    var point = routes[r].Points[p];
                    WriteSignedRegister(addr + 0, point.PathId);
                    WriteSignedRegister(addr + 1, point.DelayTicks);
                    WriteSignedRegister(addr + 2, point.RotateAngle);
                    WriteSignedRegister(addr + 3, point.RotateSpeed);
                }
            }
        }

        /// <summary>
        /// 同步当前选中路线号到寄存器
        /// </summary>
        private void SyncSelectedRouteRegister(int agvIdx)
        {
            int num = _routeControls[agvIdx].SelectedRouteNumber;
            WriteUnsignedRegister(GetAgvBase(agvIdx) + SelectedRouteOffset, (ushort)num);
        }

        /// <summary>
        /// 处理Modbus写入后的寄存器数据，更新AGV状态
        /// </summary>
        private void ProcessReceivedRegisters(int startAddr)
        {
            int agvIdx = -1;
            for (int i = 0; i < AgvCount; i++)
            {
                int s = GetAgvBase(i) + StatusOffset;
                if (startAddr >= s && startAddr < s + StatusRegisterCount) { agvIdx = i; break; }
            }
            if (agvIdx < 0) return;

            _lastUpdateTicks[agvIdx] = DateTime.Now;
            int baseAddr = GetAgvBase(agvIdx);

            var sData = new AgvStatus
            {
                X = (short)_holdingRegisters[baseAddr + 0],
                Y = (short)_holdingRegisters[baseAddr + 1],
                Yaw = (short)_holdingRegisters[baseAddr + 2],
                PathPoint = (short)_holdingRegisters[baseAddr + 3],
                PathSeg = (short)_holdingRegisters[baseAddr + 4],
                Vx = (short)_holdingRegisters[baseAddr + 5],
                Vy = (short)_holdingRegisters[baseAddr + 6],
                W = (short)_holdingRegisters[baseAddr + 7],
                Mode = (short)_holdingRegisters[baseAddr + 8],
                RunState = (short)_holdingRegisters[baseAddr + 9],
                NaviState = (short)_holdingRegisters[baseAddr + 10]
            };
            UpdateAgvData(agvIdx + 1, sData);
        }

        /// <summary>
        /// 向所有勾选的AGV下发全局指令（启动/停止/暂停）
        /// </summary>
        private async void SendCommandToAll(int cmdType)
        {
            Logger.Info($"Global command triggered: {GetCommandText(cmdType)}");
            var lines = new List<string>();
            for (int i = 0; i < AgvCount; i++)
            {
                int baseAddr = GetAgvBase(i);
                if (!_routeControls[i].CommandEnabled)
                {
                    WriteUnsignedRegister(baseAddr + CommandOffset, 0);
                    WriteUnsignedRegister(baseAddr + CommandValueOffset, 0);
                    continue;
                }
                int route = _routeControls[i].SelectedRouteNumber;
                WriteUnsignedRegister(baseAddr + SelectedRouteOffset, (ushort)route);
                WriteUnsignedRegister(baseAddr + CommandOffset, (ushort)cmdType);
                WriteUnsignedRegister(baseAddr + CommandValueOffset, 1);
                lines.Add(string.Format("    AGV #{0}: {1}, 路线 {2}", i + 1, GetCommandText(cmdType), route));
            }

            if (lines.Count == 0)
            {
                MessageBox.Show("未勾选任何 AGV。", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                Logger.Info("未勾选任何 AGV，未发送指令。");
                return;
            }
            MessageBox.Show("已下发指令:\n\n" + string.Join("\n", lines), "成功", MessageBoxButtons.OK, MessageBoxIcon.Information);

            Logger.Info("已下发指令:\n" + string.Join("\n", lines));

            // 保持高电平信号 1 秒，确保 PLC 能够稳定采集到信号变化
            await Task.Delay(1000);

            // 延时后复位指令区，确保 PLC 能通过 0->1 变化感知新指令
            for (int i = 0; i < AgvCount; i++)
            {
                int baseAddr = GetAgvBase(i);
                WriteUnsignedRegister(baseAddr + CommandOffset, 0);
                WriteUnsignedRegister(baseAddr + CommandValueOffset, 0);
            }
            Logger.Info("指令区已复位。");
        }

        /// <summary>
        /// 获取指令类型对应的中文文本
        /// </summary>
        private string GetCommandText(int type)
        {
            if (type == 1) return "启动";
            if (type == 2) return "停止";
            if (type == 3) return "暂停";
            return "未知";
        }

        /// <summary>
        /// 写有符号寄存器
        /// </summary>
        private void WriteSignedRegister(int addr, short val) => _holdingRegisters[addr] = unchecked((ushort)val);
        /// <summary>
        /// 写无符号寄存器
        /// </summary>
        private void WriteUnsignedRegister(int addr, ushort val) => _holdingRegisters[addr] = val;

        /// <summary>
        /// 同步所有AGV的路线数据和选中路线号到寄存器
        /// </summary>
        private void SyncAllRouteDataToRegisters()
        {
            for (int i = 0; i < AgvCount; i++) 
            { 
                SyncRouteLibraryToRegisters(i); 
                SyncSelectedRouteRegister(i); 
            }
        }

        /// <summary>
        /// 启动Modbus TCP服务器，监听502端口
        /// </summary>
        private void StartModbusServer()
        {
            try
            {
                _server = new TcpListener(IPAddress.Any, 502);
                _isRunning = true;
                _server.Start();
                Logger.Info("Modbus TCP Server started on port 502.");
                _listenThread = new Thread(ListenForClients) { IsBackground = true };
                _listenThread.Start();
            }
            catch (Exception ex)
            {
                Logger.Error("Failed to start Modbus Server", ex);
                MessageBox.Show("启动 Modbus 服务失败：" + ex.Message);
            }
        }

        /// <summary>
        /// 监听客户端连接，接收Modbus请求
        /// </summary>
        private void ListenForClients()
        {
            while (_isRunning)
            {
                try { var client = _server.AcceptTcpClient(); new Thread(HandleModbusClient) { IsBackground = true }.Start(client); }
                catch { if (_isRunning) Thread.Sleep(50); }
            }
        }

        /// <summary>
        /// 处理单个Modbus客户端的请求
        /// </summary>
        private void HandleModbusClient(object obj)
        {
            var client = (TcpClient)obj;
            string clientIp = client.Client.RemoteEndPoint.ToString();
            Logger.Info($"Client connected: {clientIp}");

            NetworkStream stream = client.GetStream();
            byte[] h = new byte[7];
            try
            {
                while (_isRunning)
                {
                    if (!ReadExact(stream, h, 7)) break;
                    int len = (h[4] << 8) | h[5];
                    byte[] pdu = new byte[len - 1];
                    if (!ReadExact(stream, pdu, len - 1)) break;

                    if (pdu[0] == 3) 
                        HandleReadRegisters(stream, h, pdu);
                    else if (pdu[0] == 1)
                        HandleReadCoils(stream, h, pdu);
                    else if (pdu[0] == 15)
                        HandleWriteCoils(stream, h, pdu);
                    else if (pdu[0] == 16)
                        HandleWriteRegisters(stream, h, pdu);
                }
            }
            catch (Exception ex)
            {
                Logger.Error($"Communication error with {clientIp}", ex);
            }
            finally 
            { 
                client.Close(); 
                Logger.Info($"Client disconnected: {clientIp}");
            }
        }

        /// <summary>
        /// 处理Modbus读寄存器请求
        /// </summary>
        private void HandleReadRegisters(NetworkStream s, byte[] h, byte[] p)
        {
            int addr = (p[1] << 8) | p[2], qty = (p[3] << 8) | p[4], rLen = 3 + qty * 2;
            byte[] resp = new byte[7 + rLen - 1];
            Array.Copy(h, resp, 7);
            resp[4] = (byte)(rLen >> 8); resp[5] = (byte)(rLen & 0xFF); resp[7] = 3; resp[8] = (byte)(qty * 2);
            for (int i = 0; i < qty; i++)
            {
                ushort v = (addr + i) < _holdingRegisters.Length ? _holdingRegisters[addr + i] : (ushort)0;
                resp[9 + i * 2] = (byte)(v >> 8); resp[10 + i * 2] = (byte)(v & 0xFF);
            }
            s.Write(resp, 0, resp.Length);
        }

        /// <summary>
        /// 处理Modbus读线圈请求 (FC1)
        /// </summary>
        private void HandleReadCoils(NetworkStream s, byte[] h, byte[] p)
        {
            int addr = (p[1] << 8) | p[2];
            int qty = (p[3] << 8) | p[4];
            int byteCount = (qty + 7) / 8;
            int rLen = 3 + byteCount; // UnitID(1) + FC(1) + ByteCnt(1) + Data(N)

            byte[] resp = new byte[7 + rLen - 1];
            Array.Copy(h, resp, 7);

            // 更新 MBAP 长度
            resp[4] = (byte)(rLen >> 8);
            resp[5] = (byte)(rLen & 0xFF);
            resp[7] = 1; // FC1
            resp[8] = (byte)byteCount;

            // 将 bool 数组打包成位字节流
            for (int i = 0; i < qty; i++)
            {
                if ((addr + i) < _holdingCoils.Length && _holdingCoils[addr + i])
                {
                    resp[9 + (i / 8)] |= (byte)(1 << (i % 8));
                }
            }
            s.Write(resp, 0, resp.Length);
        }

        /// <summary>
        /// 处理Modbus写线圈请求 (FC15)
        /// </summary>
        private void HandleWriteCoils(NetworkStream s, byte[] h, byte[] p)
        {
            int addr = (p[1] << 8) | p[2];
            int qty = (p[3] << 8) | p[4];
            int byteCount = p[5];

            for (int i = 0; i < qty; i++)
            {
                if ((addr + i) < _holdingCoils.Length)
                {
                    int byteIdx = 6 + (i / 8);
                    int bitIdx = i % 8;
                    _holdingCoils[addr + i] = (p[byteIdx] & (1 << bitIdx)) != 0;
                }
            }

            // 返回标准确认报文 (12字节)
            byte[] resp = new byte[12];
            Array.Copy(h, resp, 7);
            resp[4] = 0; resp[5] = 6; resp[7] = 15;
            Array.Copy(p, 1, resp, 8, 4);
            s.Write(resp, 0, resp.Length);
        }

        /// <summary>
        /// 处理Modbus写寄存器请求
        /// </summary>
        private void HandleWriteRegisters(NetworkStream s, byte[] h, byte[] p)
        {
            int addr = (p[1] << 8) | p[2], qty = (p[3] << 8) | p[4];
            for (int i = 0; i < qty; i++)
                if ((addr + i) < _holdingRegisters.Length)
                    _holdingRegisters[addr + i] = (ushort)((p[6 + i * 2] << 8) | p[7 + i * 2]);
            byte[] resp = new byte[12];
            Array.Copy(h, resp, 7);
            resp[4] = 0; resp[5] = 6; resp[7] = 16;
            Array.Copy(p, 1, resp, 8, 4);
            s.Write(resp, 0, resp.Length);
            ProcessReceivedRegisters(addr);
        }

        /// <summary>
        /// 从网络流中精确读取指定字节数
        /// </summary>
        private bool ReadExact(NetworkStream s, byte[] b, int l)
        {
            int p = 0;
            while (p < l) { int r = s.Read(b, p, l - p); if (r == 0) return false; p += r; }
            return true;
        }

        /// <summary>
        /// 超时检测定时器回调，若AGV超3秒未更新则判为离线
        /// </summary>
        private void TimeoutTimer_Tick(object sender, EventArgs e)
        {
            DateTime now = DateTime.Now;
            for (int i = 0; i < AgvCount; i++)
            {
                if ((now - _lastUpdateTicks[i]).TotalSeconds > 3)
                {
                    if (_isAgvOnline[i])
                    {
                        string[] names = { "小船", "石头", "松树" };
                        Logger.Info($"AGV #{i + 1} ({names[i]}) 离线 - 通讯超时");
                        _isAgvOnline[i] = false;
                    }
                    _agvControls[i].SetConnection(false);
                }
            }
        }

        /// <summary>
        /// 更新指定AGV的UI显示数据
        /// </summary>
        public void UpdateAgvData(int id, AgvStatus status)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(delegate { UpdateAgvData(id, status); }));
                return;
            }

            if (id >= 1 && id <= AgvCount)
            {
                if (!_isAgvOnline[id - 1])
                {
                    string[] names = { "小船", "石头", "松树" };
                    Logger.Info($"AGV #{id} ({names[id - 1]}) 在线 - 接收到数据报文");
                    _isAgvOnline[id - 1] = true;
                }
                _agvControls[id - 1].SetConnection(true);
                _agvControls[id - 1].UpdateData(status);
            }
        }

        /// <summary>
        /// 窗体关闭时，停止服务器和定时器
        /// </summary>
        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            _isRunning = false;
            if (_timeoutTimer != null)
            {
                _timeoutTimer.Stop();
            }
            if (_server != null)
            {
                _server.Stop();
            }
            base.OnFormClosing(e);
        }
    }

    /// <summary>
    /// AGV状态显示控件，显示位置、速度、状态等
    /// </summary>
    public class AgvStateControl : GroupBox
    {
        private readonly Label lblConnStatus;
        private readonly Label lblPos;
        private readonly Label lblPath;
        private readonly Label lblSpeed;
        private readonly Label lblState;

        /// <summary>
        /// 构造函数，初始化AGV状态控件
        /// </summary>
        public AgvStateControl(int id, string name = "")
        {
            Text = string.Format("AGV #{0} {1}", id, name);
            ForeColor = Color.White;
            Font = new Font("Segoe UI", 12F, FontStyle.Bold);
            Dock = DockStyle.Fill;
            Margin = new Padding(8);
            Padding = new Padding(15);
            BackColor = Color.FromArgb(28, 30, 36);

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 5,
                BackColor = Color.Transparent
            };
            Controls.Add(layout);

            lblConnStatus = CreateLabel("状态: [等待连接...]", Color.Silver, 11F, FontStyle.Bold);
            lblPos = CreateLabel("位置: X=0.00m Y=0.00m Yaw=0.0°", Color.Gainsboro, 10.5F, FontStyle.Regular);
            lblPath = CreateLabel("路径: 点 0 / 段 0", Color.Gainsboro, 10.5F, FontStyle.Regular);
            lblSpeed = CreateLabel("速度: Vf=0.000m/s Vl=0.000m/s Va=0.0°/s", Color.Gainsboro, 10.5F, FontStyle.Regular);
            lblState = CreateLabel("模式: 未知 (未知) 导航: 未知", Color.Gainsboro, 10.5F, FontStyle.Regular);

            layout.Controls.Add(lblConnStatus);
            layout.Controls.Add(lblPos);
            layout.Controls.Add(lblPath);
            layout.Controls.Add(lblSpeed);
            layout.Controls.Add(lblState);
        }

        /// <summary>
        /// 创建带样式的Label
        /// </summary>
        private Label CreateLabel(string text, Color color, float fontSize, FontStyle style)
        {
            return new Label
            {
                Text = text,
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", fontSize, style),
                ForeColor = color,
                TextAlign = ContentAlignment.MiddleLeft
            };
        }

        /// <summary>
        /// 设置连接状态显示（在线/离线）
        /// </summary>
        public void SetConnection(bool connected)
        {
            if (connected)
            {
                lblConnStatus.Text = "状态: [在线]";
                lblConnStatus.ForeColor = Color.FromArgb(79, 214, 121);
            }
            else
            {
                lblConnStatus.Text = "状态: [离线]";
                lblConnStatus.ForeColor = Color.FromArgb(240, 92, 92);
            }
        }

        /// <summary>
        /// 更新AGV状态数据到控件
        /// </summary>
        public void UpdateData(AgvStatus s)
        {
            // Yaw单位已改为0.1度/bit，直接显示
            lblPos.Text = string.Format("位置: X={0:F2}m Y={1:F2}m Yaw={2:F1}°", s.X / 100.0, s.Y / 100.0, s.Yaw / 10.0);
            lblPath.Text = string.Format("路径: 点 {0} / 段 {1}", s.PathPoint, s.PathSeg);
            lblSpeed.Text = string.Format("速度: Vf={0:F3}m/s Va={1:F1}°/s", s.Vx / 1000.0, s.W * 0.0573);

            string mode = "未知";
            if (s.Mode == 0) mode = "手动";
            else if (s.Mode == 1) mode = "自动";
            else if (s.Mode == 2) mode = "导航";

            string run = "未知";
            if (s.RunState == 0) run = "停止";
            else if (s.RunState == 1) run = "运行";
            else if (s.RunState == 2) run = "暂停";

            lblState.Text = string.Format("模式: {0} ({1})", mode, run);
        }
    }

    /// <summary>
    /// 路线选择控件，包含路线下拉、编辑按钮等
    /// </summary>
    public class RouteSelectionControl : GroupBox
    {
        private readonly CheckBox chkEnabled;
        private readonly ComboBox cmbRoute;
        private readonly Label lblSummary;
        private readonly Button btnEdit;

        public event EventHandler EditClicked;
        public event EventHandler RouteSelectionChanged;

        /// <summary>
        /// 构造函数，初始化路线选择控件
        /// </summary>
        public RouteSelectionControl(int agvId, int routeCount, string name = "")
        {
            Text = string.Format("AGV #{0} {1} 工艺路线", agvId, name);
            Dock = DockStyle.Fill;
            Margin = new Padding(8);
            Padding = new Padding(14);
            Font = new Font("Segoe UI", 11F, FontStyle.Bold);
            ForeColor = Color.White;
            BackColor = Color.FromArgb(36, 39, 46);

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 108F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 34F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 38F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            Controls.Add(layout);

            chkEnabled = new CheckBox
            {
                Text = "加入本次全局指令",
                Dock = DockStyle.Fill,
                ForeColor = Color.White,
                Checked = true, // 默认勾选
                Font = new Font("Segoe UI", 10F, FontStyle.Regular)
            };

            cmbRoute = new ComboBox
            {
                Dock = DockStyle.Fill,
                DropDownStyle = ComboBoxStyle.DropDownList,
                Font = new Font("Segoe UI", 11F, FontStyle.Regular),
                BackColor = Color.White,
                ForeColor = Color.Black, // 明确前景色
                FlatStyle = FlatStyle.Standard // 改回标准样式通常更稳定
            };
            cmbRoute.Items.Clear(); // 确保加载前清空
            for (int i = 1; i <= routeCount; i++)
            {
                cmbRoute.Items.Add("路线 " + i);
            }
            if (cmbRoute.Items.Count > 0) cmbRoute.SelectedIndex = 0; // 初始选择第一项
            cmbRoute.SelectedIndex = 0;
            cmbRoute.SelectedIndexChanged += delegate
            {
                UpdateSummary();
                if (RouteSelectionChanged != null)
                {
                    RouteSelectionChanged(this, EventArgs.Empty);
                }
            };

            btnEdit = new Button
            {
                Text = "编辑路线",
                Dock = DockStyle.Fill,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(78, 163, 255),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 11F, FontStyle.Bold)
            };
            btnEdit.FlatAppearance.BorderSize = 0;
            btnEdit.Click += delegate
            {
                if (EditClicked != null)
                {
                    EditClicked(this, EventArgs.Empty);
                }
            };

            lblSummary = new Label
            {
                Text = "当前选择: 路线 1",
                Dock = DockStyle.Fill,
                ForeColor = Color.FromArgb(178, 190, 195),
                Font = new Font("Segoe UI", 10F, FontStyle.Italic),
                TextAlign = ContentAlignment.BottomLeft
            };

            layout.Controls.Add(chkEnabled, 0, 0);
            layout.SetColumnSpan(chkEnabled, 2);
            layout.Controls.Add(cmbRoute, 0, 1);
            layout.Controls.Add(btnEdit, 1, 1);
            layout.Controls.Add(lblSummary, 0, 2);
            layout.SetColumnSpan(lblSummary, 2);

            UpdateSummary();
        }

        /// <summary>
        /// 是否勾选参与全局指令
        /// </summary>
        public bool CommandEnabled { get { return chkEnabled.Checked; } }
        /// <summary>
        /// 当前选中的路线编号
        /// </summary>
        public int SelectedRouteNumber { get { return cmbRoute.SelectedIndex + 1; } }

        /// <summary>
        /// 更新下方路线摘要显示
        /// </summary>
        private void UpdateSummary()
        {
            lblSummary.Text = string.Format("当前已选: {0}", cmbRoute.SelectedItem);
        }
    }

    /// <summary>
    /// 路线编辑器窗口，支持编辑、保存、加载工艺路线
    /// </summary>
    public class RouteEditorForm : Form
    {
        private List<ProcessRoute> _routes;
        private readonly string _persistFile;
        private Action _saveCallback;
        private int _currentRouteIndex = -1;

        private ListBox lstRoutes;
        private DataGridView gridPoints;

        /// <summary>
        /// 构造函数，初始化路线编辑器，自动加载持久化文件
        /// </summary>
        /// <param name="agvId">AGV编号</param>
        /// <param name="routes">路线列表</param>
        /// <param name="name">AGV名称</param>
        /// <param name="saveCallback">保存回调</param>
        public RouteEditorForm(int agvId, List<ProcessRoute> routes, string name, Action saveCallback)
        {
            _routes = routes;
            _saveCallback = saveCallback;
            _persistFile = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, $"agv_routes_{agvId}.json");

            // 启动时自动加载持久化文件
            if (System.IO.File.Exists(_persistFile))
            {
                try
                {
                    var json = System.IO.File.ReadAllText(_persistFile);
                    var loaded = Newtonsoft.Json.JsonConvert.DeserializeObject<List<ProcessRoute>>(json);
                    if (loaded != null && loaded.Count == _routes.Count)
                    {
                        for (int i = 0; i < _routes.Count; i++)
                        {
                            _routes[i].RouteNumber = loaded[i].RouteNumber;
                            for (int j = 0; j < _routes[i].Points.Count && j < loaded[i].Points.Count; j++)
                            {
                                _routes[i].Points[j] = loaded[i].Points[j];
                            }
                        }
                    }
                }
                catch { /* ignore file errors */ }
            }

            Text = string.Format("AGV #{0} {1} 工艺路线编辑器 (20路线 × 10点位)", agvId, name);
            Size = new Size(1100, 780);
            StartPosition = FormStartPosition.CenterParent;
            BackColor = Color.FromArgb(41, 45, 54);
            ForeColor = Color.White;
            Font = new Font("Microsoft YaHei", 12F, FontStyle.Regular);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                Padding = new Padding(20),
                BackColor = Color.FromArgb(28, 30, 36)
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 220F));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 72F));
            Controls.Add(mainLayout);

            var leftPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(32, 36, 43),
                Padding = new Padding(15)
            };
            var leftTitle = new Label
            {
                Text = "工艺路线库",
                Dock = DockStyle.Top,
                Height = 40,
                Font = new Font("Microsoft YaHei", 14F, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 168, 255)
            };
            lstRoutes = new ListBox
            {
                Dock = DockStyle.Fill,
                Font = new Font("Microsoft YaHei", 12F, FontStyle.Regular),
                BackColor = Color.FromArgb(53, 59, 72),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.None,
                ItemHeight = 40,
                DrawMode = DrawMode.OwnerDrawFixed
            };
            lstRoutes.DrawItem += (s, e) =>
            {
                if (e.Index < 0) return;
                bool isSelected = (e.State & DrawItemState.Selected) == DrawItemState.Selected;
                e.Graphics.FillRectangle(new SolidBrush(isSelected ? Color.FromArgb(0, 151, 230) : Color.FromArgb(53, 59, 72)), e.Bounds);
                string text = lstRoutes.Items[e.Index].ToString();
                TextRenderer.DrawText(e.Graphics, text, lstRoutes.Font, e.Bounds, Color.White, TextFormatFlags.VerticalCenter | TextFormatFlags.Left);
            };
            for (int i = 0; i < _routes.Count; i++)
            {
                lstRoutes.Items.Add(string.Format(" 路线 #{0:D2}", _routes[i].RouteNumber));
            }
            lstRoutes.SelectedIndexChanged += LstRoutes_SelectedIndexChanged;
            leftPanel.Controls.Add(lstRoutes);
            leftPanel.Controls.Add(leftTitle);
            mainLayout.Controls.Add(leftPanel, 0, 0);

            var rightPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(32, 36, 43),
                Padding = new Padding(20)
            };

            var infoPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 40,
                BackColor = Color.Transparent
            };
            var infoLabel = new Label
            {
                Dock = DockStyle.Fill,
                Text = "配置说明：每条路线最多包含10个路径点（点01 到 点10），路径点ID 是指地图上的路径点 P 的序号",
                ForeColor = Color.FromArgb(178, 190, 195),
                Font = new Font("Microsoft YaHei", 11F, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft
            };
            infoPanel.Controls.Add(infoLabel);

            gridPoints = new DataGridView
            {
                Dock = DockStyle.Fill,
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = false,
                AllowUserToResizeRows = false,
                BackgroundColor = Color.FromArgb(41, 45, 54),
                BorderStyle = BorderStyle.None,
                ColumnHeadersHeight = 55,
                RowHeadersWidth = 100,
                Font = new Font("Microsoft YaHei", 12F, FontStyle.Bold),
                EnableHeadersVisualStyles = false,
                SelectionMode = DataGridViewSelectionMode.CellSelect,
                MultiSelect = false,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                GridColor = Color.FromArgb(63, 71, 89)
            };
            gridPoints.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(47, 54, 64);
            gridPoints.ColumnHeadersDefaultCellStyle.ForeColor = Color.FromArgb(220, 221, 225);
            gridPoints.ColumnHeadersDefaultCellStyle.Font = new Font("Microsoft YaHei", 12F, FontStyle.Bold);
            gridPoints.DefaultCellStyle.BackColor = Color.FromArgb(53, 59, 72);
            gridPoints.DefaultCellStyle.ForeColor = Color.White;
            gridPoints.DefaultCellStyle.Font = new Font("Consolas", 14F);
            gridPoints.DefaultCellStyle.SelectionBackColor = Color.FromArgb(0, 151, 230);
            gridPoints.DefaultCellStyle.SelectionForeColor = Color.White;
            gridPoints.RowHeadersDefaultCellStyle.BackColor = Color.FromArgb(47, 54, 64);
            gridPoints.RowHeadersDefaultCellStyle.ForeColor = Color.FromArgb(178, 190, 195);
            gridPoints.RowHeadersDefaultCellStyle.Font = new Font("Microsoft YaHei", 11F, FontStyle.Bold);
            gridPoints.CellPainting += gridPoints_CellPainting;

            gridPoints.Columns.Add("PathId", "路径点 ID(地图)");
            gridPoints.Columns.Add("DelayTicks", "启动延时(0.1秒)");
            gridPoints.Columns.Add("RotateAngle", "旋转角度(度)");
            gridPoints.Columns.Add("RotateSpeed", "旋转速度(度/秒)");
            for (int i = 0; i < 10; i++)
            {
                int rowIndex = gridPoints.Rows.Add();
                gridPoints.Rows[rowIndex].Height = 45;
                gridPoints.Rows[rowIndex].HeaderCell.Value = string.Format(" 点 {0:D2}", i + 1);
            }
            rightPanel.Controls.Add(gridPoints);
            rightPanel.Controls.Add(infoPanel);
            mainLayout.Controls.Add(rightPanel, 1, 0);

            var bottomPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.RightToLeft,
                BackColor = Color.Transparent,
                Padding = new Padding(0, 8, 0, 0)
            };
            var btnClose = new Button
            {
                Text = "关闭窗口",
                Width = 140,
                Height = 46,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(72, 84, 96),
                ForeColor = Color.White,
                Font = new Font("Microsoft YaHei", 12F, FontStyle.Bold)
            };
            btnClose.FlatAppearance.BorderSize = 0;
            btnClose.Click += delegate { Close(); };

            var btnSave = new Button
            {
                Text = "保存并下发路线",
                Width = 180,
                Height = 46,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 151, 230),
                ForeColor = Color.White,
                Font = new Font("Microsoft YaHei", 12F, FontStyle.Bold)
            };
            btnSave.FlatAppearance.BorderSize = 0;
            btnSave.Click += BtnSave_Click;

            bottomPanel.Controls.Add(btnClose);
            bottomPanel.Controls.Add(btnSave);
            mainLayout.Controls.Add(bottomPanel, 1, 1);

            lstRoutes.SelectedIndex = 0;
            FormClosing += RouteEditorForm_FormClosing;
        }

        /// <summary>
        /// 自定义单元格绘制，增强表格美观
        /// </summary>
        private void gridPoints_CellPainting(object sender, DataGridViewCellPaintingEventArgs e)
        {
            if (e.RowIndex >= 0 && e.ColumnIndex >= 0)
            {
                e.Paint(e.CellBounds, DataGridViewPaintParts.All);
                ControlPaint.DrawBorder(e.Graphics, e.CellBounds, Color.FromArgb(63, 71, 89), ButtonBorderStyle.Solid);
                e.Handled = true;
            }
        }

        /// <summary>
        /// 路线切换时保存当前并加载新路线
        /// </summary>
        private void LstRoutes_SelectedIndexChanged(object sender, EventArgs e)
        {
            SaveCurrentRoute();
            LoadRoute(lstRoutes.SelectedIndex);
        }

        /// <summary>
        /// 加载指定路线到表格
        /// </summary>
        private void LoadRoute(int routeIndex)
        {
            if (routeIndex < 0 || routeIndex >= _routes.Count)
            {
                return;
            }

            _currentRouteIndex = routeIndex;
            ProcessRoute route = _routes[routeIndex];
            for (int i = 0; i < route.Points.Count; i++)
            {
                gridPoints.Rows[i].Cells[0].Value = route.Points[i].PathId;
                gridPoints.Rows[i].Cells[1].Value = route.Points[i].DelayTicks;
                gridPoints.Rows[i].Cells[2].Value = route.Points[i].RotateAngle;
                gridPoints.Rows[i].Cells[3].Value = route.Points[i].RotateSpeed;
            }
            // 设置表格左上角表头为当前工艺路线名称
            if (routeIndex >= 0 && routeIndex < lstRoutes.Items.Count)
            {
                gridPoints.TopLeftHeaderCell.Value = $"{lstRoutes.Items[routeIndex]}";
            }
        }

        /// <summary>
        /// 保存当前表格数据到路线对象
        /// </summary>
        private void SaveCurrentRoute()
        {
            if (_currentRouteIndex < 0 || _currentRouteIndex >= _routes.Count)
            {
                return;
            }

            ProcessRoute route = _routes[_currentRouteIndex];
            for (int i = 0; i < route.Points.Count; i++)
            {
                route.Points[i].PathId = ClampToShort(ParseCell(gridPoints.Rows[i].Cells[0].Value), 0, 1000);
                route.Points[i].DelayTicks = ClampToShort(ParseCell(gridPoints.Rows[i].Cells[1].Value), 0, short.MaxValue);
                route.Points[i].RotateAngle = ClampToShort(ParseCell(gridPoints.Rows[i].Cells[2].Value), short.MinValue, short.MaxValue);
                route.Points[i].RotateSpeed = ClampToShort(ParseCell(gridPoints.Rows[i].Cells[3].Value), 0, short.MaxValue);
            }
        }

        /// <summary>
        /// 解析单元格值为int，异常返回0
        /// </summary>
        private int ParseCell(object value)
        {
            int result;
            if (!int.TryParse(Convert.ToString(value), out result))
            {
                result = 0;
            }
            return result;
        }

        /// <summary>
        /// 限制数值在short范围内
        /// </summary>
        private short ClampToShort(int value, int min, int max)
        {
            if (value < min)
            {
                value = min;
            }
            if (value > max)
            {
                value = max;
            }
            return (short)value;
        }

        /// <summary>
        /// 保存按钮点击，持久化路线到文件并同步到寄存器
        /// </summary>
        private void BtnSave_Click(object sender, EventArgs e)
        {
            SaveCurrentRoute();
            // 持久化到文件
            try
            {
                var json = Newtonsoft.Json.JsonConvert.SerializeObject(_routes, Newtonsoft.Json.Formatting.Indented);
                System.IO.File.WriteAllText(_persistFile, json);
                Logger.Info($"Routes saved to file: {_persistFile}");
            }
            catch (Exception ex)
            {
                Logger.Error("Failed to save routes to file", ex);
                MessageBox.Show("保存到文件失败：" + ex.Message, "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (_saveCallback != null)
            {
                _saveCallback();
            }
            MessageBox.Show("工艺路线已保存，并同步到PLC可读取的寄存器区。", "保存成功", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }

        /// <summary>
        /// 窗口关闭时自动保存当前路线并同步
        /// </summary>
        private void RouteEditorForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            SaveCurrentRoute();
            if (_saveCallback != null)
            {
                _saveCallback();
            }
        }
    }
}
