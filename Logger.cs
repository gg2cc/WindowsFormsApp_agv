using System;
using System.IO;

namespace WindowsFormsApp_agv
{
    public static class Logger
    {
        private static readonly string LogDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs");
        private static readonly object _lock = new object();

        static Logger()
        {
            if (!Directory.Exists(LogDirectory))
                Directory.CreateDirectory(LogDirectory);
        }

        public static void Info(string message) => Write("INFO", message);
        public static void Error(string message, Exception ex = null)
            => Write("ERROR", ex != null ? $"{message} | EX: {ex.Message}" : message);

        private static void Write(string level, string message)
        {
            try
            {
                lock (_lock)
                {
                    string fileName = Path.Combine(LogDirectory, $"{DateTime.Now:yyyyMMdd}.log");
                    string logLine = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [{level}] {message}{Environment.NewLine}";
                    File.AppendAllText(fileName, logLine);
                }
            }
            catch { /* 忽略日志写入本身的异常以防程序崩溃 */ }
        }
    }
}