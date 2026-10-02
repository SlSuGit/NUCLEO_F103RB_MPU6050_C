#:property TargetFramework=net10.0-windows
#:property UseWindowsForms=true
#:property OutputType=WinExe
#:property PublishTrimmed=false
#:package System.IO.Ports@10.0.12

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new UartMonitorForm());
    }
}

internal sealed class UartMonitorForm : Form
{
    #region Ustawienia do edycji

    // Kalibracja w g: wynik = (odczyt - bias) * skala. X i Y sa neutralne.
    private const double AccelerometerXBiasG = 0.0;
    private const double AccelerometerXScale = 1.0;
    private const double AccelerometerYBiasG = 0.0;
    private const double AccelerometerYScale = 1.0;
    private const double AccelerometerZBiasG = -0.178;
    private const double AccelerometerZScale = 1.0 / 1.018;

    // Tempo odswiezania okna i maksymalna liczba wierszy historii.
    private const int DisplayRefreshIntervalMs = 50;
    private const int MaximumDisplayedFrames = 1500;
    private const int MaximumPartialLineLength = 4096;

    #endregion

    #region Kolory okna

    private static readonly Color WindowColor = Color.FromArgb(243, 246, 245);
    private static readonly Color InkColor = Color.FromArgb(29, 48, 50);
    private static readonly Color MutedColor = Color.FromArgb(112, 129, 127);
    private static readonly Color AccentColor = Color.FromArgb(8, 126, 120);
    private static readonly Color BorderColor = Color.FromArgb(216, 225, 223);

    #endregion

    #region Kontrolki i stan aplikacji

    private readonly ComboBox _portComboBox = new();
    private readonly ComboBox _baudComboBox = new();
    private readonly ComboBox _accelRangeComboBox = new();
    private readonly ComboBox _gyroRangeComboBox = new();
    private readonly Button _connectButton = new();
    private readonly Button _refreshButton = new();
    private readonly Button _applyRangesButton = new();
    private readonly Button _clearButton = new();
    private readonly ListBox _receivedListBox = new();
    private readonly Label _statusIndicator = new();
    private readonly Label _statusText = new();
    private readonly Label _statusDetail = new();
    private readonly Label _frameCountText = new();
    private readonly Label _latestFrameText = new();
    private readonly Label _lastReceivedText = new();
    private readonly AccelerationPlot _accelerationPlot = new();
    private readonly System.Windows.Forms.Timer _displayTimer = new() { Interval = DisplayRefreshIntervalMs };
    private readonly object _receiveLock = new();
    private readonly StringBuilder _partialLine = new();
    private readonly Queue<ReceivedFrame> _pendingFrames = new();
    private SerialPort? _serialPort;
    private long _receivedFrameCount;
    private bool _rangeCommandPending;
    // Czujnik sam potwierdza zakres i czulosc po polaczeniu lub zmianie ustawien.
    private double _accelerometerCountsPerG = 16384.0;

    #endregion

    #region Dostepne zakresy czujnika

    private static readonly RangeOption[] AccelerometerRanges =
    {
        new("±2 g", 2, 16384.0),
        new("±4 g", 4, 8192.0),
        new("±8 g", 8, 4096.0),
        new("±16 g", 16, 2048.0)
    };

    private static readonly RangeOption[] GyroscopeRanges =
    {
        new("±250 °/s", 250, 131.0),
        new("±500 °/s", 500, 65.5),
        new("±1000 °/s", 1000, 32.8),
        new("±2000 °/s", 2000, 16.4)
    };

    #endregion

    #region Format ramek

    private readonly record struct ReceivedFrame(DateTime Timestamp, string Line);
    private readonly record struct RangeOption(string Label, int FullScale, double Sensitivity)
    {
        public override string ToString() => Label;
    }

    private readonly record struct Mpu6050Frame(
        int AccelX, int AccelY, int AccelZ,
        int TemperatureRaw,
        int GyroX, int GyroY, int GyroZ)
    {
        public double TemperatureCelsius => TemperatureRaw / 340.0 + 36.53;
    }

    #endregion

    #region Budowanie okna

    public UartMonitorForm()
    {
        Text = "IMU Flight View | UART Monitor";
        ClientSize = new Size(1080, 680);
        MinimumSize = new Size(820, 540);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = WindowColor;
        ForeColor = InkColor;
        Font = new Font("Segoe UI", 9.5f);
        AutoScaleMode = AutoScaleMode.Dpi;

        Controls.Add(BuildLayout());
        Load += (_, _) => RefreshPorts();
        Load += (_, _) => _displayTimer.Start();
        _displayTimer.Tick += (_, _) => RefreshReceivedFrames();
        FormClosing += (_, _) =>
        {
            _displayTimer.Stop();
            _displayTimer.Dispose();
            DisconnectPort();
        };
    }

    private Control BuildLayout()
    {
        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 3,
            BackColor = WindowColor,
            Margin = Padding.Empty
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 148));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        layout.Controls.Add(BuildStatusBar(), 0, 0);
        layout.Controls.Add(BuildConnectionBar(), 0, 1);
        layout.Controls.Add(BuildContent(), 0, 2);
        return layout;
    }

    private Control BuildStatusBar()
    {
        var bar = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = Color.FromArgb(34, 54, 56),
            Padding = new Padding(24, 0, 24, 0)
        };

        _statusIndicator.Text = "●";
        _statusIndicator.Font = new Font("Segoe UI Symbol", 12, FontStyle.Bold);
        _statusIndicator.ForeColor = Color.FromArgb(217, 109, 97);
        _statusIndicator.AutoSize = true;
        _statusIndicator.Location = new Point(24, 18);

        _statusText.Text = "ROZŁĄCZONO";
        _statusText.Font = new Font("Segoe UI", 9, FontStyle.Bold);
        _statusText.ForeColor = Color.FromArgb(242, 246, 245);
        _statusText.AutoSize = true;
        _statusText.Location = new Point(43, 11);

        _statusDetail.Text = "Wybierz port szeregowy";
        _statusDetail.Font = new Font("Segoe UI", 8.5f);
        _statusDetail.ForeColor = Color.FromArgb(175, 192, 189);
        _statusDetail.AutoSize = true;
        _statusDetail.Location = new Point(43, 31);

        var appTag = new Label
        {
            Text = "UART MONITOR  /  8N1",
            AutoSize = true,
            ForeColor = Color.FromArgb(175, 192, 189),
            Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
            Anchor = AnchorStyles.Top | AnchorStyles.Right,
            Location = new Point(Width - 200, 20)
        };
        bar.Resize += (_, _) => appTag.Location = new Point(bar.ClientSize.Width - appTag.Width - 24, 20);

        bar.Controls.Add(_statusIndicator);
        bar.Controls.Add(_statusText);
        bar.Controls.Add(_statusDetail);
        bar.Controls.Add(appTag);
        return bar;
    }

    private Control BuildConnectionBar()
    {
        var bar = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = Color.White
        };

        ConfigureComboBox(_portComboBox, 160);
        ConfigureComboBox(_baudComboBox, 160);
        _baudComboBox.DropDownStyle = ComboBoxStyle.DropDownList;
        _baudComboBox.Items.AddRange(new object[] { 
                            "300",
                            "600",
                            "1200",
                            "2400",
                            "4800",
                            "9600",
                            "14400",
                            "19200",
                            "28800",
                            "38400",
                            "57600",
                            "115200",
                            "230400",
                            "460800",
                            "921600" });
        _baudComboBox.SelectedItem = "921600";

        foreach (var range in AccelerometerRanges)
        {
            _accelRangeComboBox.Items.Add(range);
        }
        _accelRangeComboBox.SelectedIndex = 0;

        foreach (var range in GyroscopeRanges)
        {
            _gyroRangeComboBox.Items.Add(range);
        }
        _gyroRangeComboBox.SelectedIndex = 0;

        _refreshButton.Text = "Odśwież";
        ConfigureSecondaryButton(_refreshButton, 100);
        _refreshButton.Margin = new Padding(0, 17, 0, 0);
        _refreshButton.Click += (_, _) => RefreshPorts();

        _connectButton.Text = "Otwórz port";
        ConfigurePrimaryButton(_connectButton, 142);
        _connectButton.Margin = new Padding(0, 17, 0, 0);
        _connectButton.Click += (_, _) => ToggleConnection();

        _applyRangesButton.Text = "Zastosuj";
        ConfigureSecondaryButton(_applyRangesButton, 100);
        _applyRangesButton.Height = 32;
        _applyRangesButton.Margin = new Padding(0, 13, 0, 0);
        _applyRangesButton.Enabled = false;
        _applyRangesButton.Click += (_, _) => ApplySelectedRanges();

        SetRangeControlsEnabled(false);

        var rows = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 2,
            Padding = new Padding(24, 8, 24, 8),
            BackColor = Color.White
        };
        rows.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 190));
        rows.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 190));
        rows.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 112));
        rows.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 142));
        rows.RowStyles.Add(new RowStyle(SizeType.Absolute, 74));
        rows.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
        rows.Controls.Add(BuildField("Nr portu", _portComboBox, 176), 0, 0);
        rows.Controls.Add(BuildField("Prędkość", _baudComboBox, 176), 1, 0);
        rows.Controls.Add(_refreshButton, 2, 0);
        rows.Controls.Add(_connectButton, 3, 0);
        rows.Controls.Add(BuildCompactField("Akcelerometr", _accelRangeComboBox, 176), 0, 1);
        rows.Controls.Add(BuildCompactField("Żyroskop", _gyroRangeComboBox, 176), 1, 1);
        rows.Controls.Add(_applyRangesButton, 2, 1);
        bar.Controls.Add(rows);
        return bar;
    }

    private Control BuildCompactField(string labelText, Control editor, int width)
    {
        var field = new TableLayoutPanel
        {
            Width = width,
            Height = 40,
            RowCount = 2,
            ColumnCount = 1,
            Margin = new Padding(0, 0, 14, 0),
            BackColor = Color.White
        };
        field.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        field.RowStyles.Add(new RowStyle(SizeType.Absolute, 17));
        field.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var label = new Label
        {
            Text = labelText,
            Dock = DockStyle.Fill,
            ForeColor = MutedColor,
            TextAlign = ContentAlignment.MiddleLeft
        };
        ConfigureComboBox(editor as ComboBox ?? throw new ArgumentException("Range editor must be a ComboBox."), width - 2);
        editor.Dock = DockStyle.Fill;
        field.Controls.Add(label, 0, 0);
        field.Controls.Add(editor, 0, 1);
        return field;
    }

    private void SetRangeControlsEnabled(bool enabled)
    {
        _accelRangeComboBox.Enabled = enabled;
        _gyroRangeComboBox.Enabled = enabled;
        _applyRangesButton.Enabled = enabled && !_rangeCommandPending;
    }

    private void ApplySelectedRanges()
    {
        if (_serialPort?.IsOpen != true ||
            _accelRangeComboBox.SelectedItem is not RangeOption accelRange ||
            _gyroRangeComboBox.SelectedItem is not RangeOption gyroRange)
        {
            return;
        }

        try
        {
            _serialPort.WriteLine($"RANGE,{accelRange.FullScale},{gyroRange.FullScale}");
            _rangeCommandPending = true;
            _accelRangeComboBox.Enabled = false;
            _gyroRangeComboBox.Enabled = false;
            _applyRangesButton.Enabled = false;
            _statusDetail.Text = "Oczekiwanie na potwierdzenie zakresu";
        }
        catch (Exception exception) when (exception is InvalidOperationException || exception is IOException)
        {
            _statusDetail.Text = "Nie udało się wysłać ustawień zakresu";
            MessageBox.Show(this, exception.Message, "Błąd portu", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    private void QueryCurrentRanges()
    {
        _serialPort?.WriteLine("RANGE?");
    }

    private static bool TryParseRangeConfirmation(
        string line,
        out RangeOption accelRange,
        out RangeOption gyroRange)
    {
        accelRange = default;
        gyroRange = default;
        var fields = line.Split(',');
        if (fields.Length != 3 ||
            !string.Equals(fields[0], "RANGE_OK", StringComparison.Ordinal) ||
            !int.TryParse(fields[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var accelFullScale) ||
            !int.TryParse(fields[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var gyroFullScale))
        {
            return false;
        }

        foreach (var range in AccelerometerRanges)
        {
            if (range.FullScale == accelFullScale)
            {
                accelRange = range;
                break;
            }
        }
        foreach (var range in GyroscopeRanges)
        {
            if (range.FullScale == gyroFullScale)
            {
                gyroRange = range;
                break;
            }
        }

        return accelRange.FullScale != 0 && gyroRange.FullScale != 0;
    }

    private void SetConfirmedRanges(RangeOption accelRange, RangeOption gyroRange)
    {
        _accelerometerCountsPerG = accelRange.Sensitivity;
        _accelerationPlot.SetRange(accelRange.FullScale);
        _accelRangeComboBox.SelectedItem = accelRange;
        _gyroRangeComboBox.SelectedItem = gyroRange;
        _rangeCommandPending = false;
        SetRangeControlsEnabled(_serialPort?.IsOpen == true);

        if (_serialPort?.IsOpen == true)
        {
            _statusDetail.Text = $"{_serialPort.PortName}  ·  {_serialPort.BaudRate} baud  ·  {accelRange.Label}  ·  {gyroRange.Label}";
        }
    }

    private Control BuildField(string labelText, Control editor, int width)
    {
        var field = new TableLayoutPanel
        {
            Width = width,
            Height = 74,
            RowCount = 2,
            ColumnCount = 1,
            Margin = new Padding(0, 0, 14, 0),
            BackColor = Color.White
        };
        field.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        field.RowStyles.Add(new RowStyle(SizeType.Absolute, 24));
        field.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));

        var label = new Label
        {
            Text = labelText,
            Dock = DockStyle.Fill,
            ForeColor = MutedColor,
            TextAlign = ContentAlignment.MiddleLeft
        };
        editor.Dock = DockStyle.Fill;
        field.Controls.Add(label, 0, 0);
        field.Controls.Add(editor, 0, 1);
        return field;
    }

    private Control BuildContent()
    {
        var content = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            Padding = new Padding(24, 18, 24, 24),
            BackColor = WindowColor
        };
        content.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 390));
        content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        content.Controls.Add(BuildReceivePanel(), 0, 0);
        content.Controls.Add(BuildLatestFramePanel(), 1, 0);
        return content;
    }

    private Control BuildReceivePanel()
    {
        var panel = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = Color.White,
            BorderStyle = BorderStyle.FixedSingle,
            Padding = new Padding(1)
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 3,
            BackColor = Color.White
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 1));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var heading = new Panel { Dock = DockStyle.Fill, BackColor = Color.White, Padding = new Padding(15, 0, 12, 0) };
        var title = new Label
        {
            Text = "ODEBRANE RAMKI",
            AutoSize = true,
            ForeColor = MutedColor,
            Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
            Location = new Point(15, 17)
        };
        _frameCountText.Text = "0";
        _frameCountText.AutoSize = true;
        _frameCountText.ForeColor = AccentColor;
        _frameCountText.Font = new Font("Segoe UI", 8.5f, FontStyle.Bold);
        _frameCountText.Location = new Point(145, 17);

        _clearButton.Text = "Wyczyść";
        ConfigureSecondaryButton(_clearButton, 82);
        _clearButton.Height = 30;
        _clearButton.Anchor = AnchorStyles.Top | AnchorStyles.Right;
        _clearButton.Location = new Point(heading.ClientSize.Width - 94, 8);
        _clearButton.Click += (_, _) => ClearReceivedFrames();
        heading.Resize += (_, _) => _clearButton.Location = new Point(heading.ClientSize.Width - 94, 8);
        heading.Controls.Add(title);
        heading.Controls.Add(_frameCountText);
        heading.Controls.Add(_clearButton);

        _receivedListBox.Dock = DockStyle.Fill;
        _receivedListBox.BorderStyle = BorderStyle.None;
        _receivedListBox.BackColor = Color.White;
        _receivedListBox.ForeColor = InkColor;
        _receivedListBox.Font = new Font("Consolas", 9.5f);
        _receivedListBox.HorizontalScrollbar = true;
        _receivedListBox.IntegralHeight = false;

        var divider = new Panel { Dock = DockStyle.Fill, BackColor = BorderColor, Height = 1 };
        layout.Controls.Add(heading, 0, 0);
        layout.Controls.Add(divider, 0, 1);
        layout.Controls.Add(_receivedListBox, 0, 2);
        panel.Controls.Add(layout);
        return panel;
    }

    private Control BuildLatestFramePanel()
    {
        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 5,
            BackColor = WindowColor,
            Padding = new Padding(32, 12, 8, 8)
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 26));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 78));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 22));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var heading = new Label
        {
            Text = "OSTATNIA RAMKA",
            Dock = DockStyle.Fill,
            ForeColor = MutedColor,
            Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
            TextAlign = ContentAlignment.MiddleLeft,
            Margin = Padding.Empty
        };
        _latestFrameText.Text = "—";
        _latestFrameText.Dock = DockStyle.Fill;
        _latestFrameText.ForeColor = InkColor;
        _latestFrameText.Font = new Font("Consolas", 11, FontStyle.Regular);
        _latestFrameText.AutoEllipsis = true;
        _latestFrameText.Margin = Padding.Empty;

        _lastReceivedText.Text = "Brak odebranych danych";
        _lastReceivedText.Dock = DockStyle.Fill;
        _lastReceivedText.ForeColor = MutedColor;
        _lastReceivedText.Font = new Font("Segoe UI", 9);
        _lastReceivedText.TextAlign = ContentAlignment.MiddleLeft;
        _lastReceivedText.Margin = Padding.Empty;

        var plotHeading = new Label
        {
            Text = "PRZEBIEG AKCELEROMETRU · OSTATNIE 3 MIN",
            Dock = DockStyle.Fill,
            ForeColor = MutedColor,
            Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
            TextAlign = ContentAlignment.MiddleLeft,
            Margin = Padding.Empty
        };

        _accelerationPlot.Dock = DockStyle.Fill;
        _accelerationPlot.Margin = Padding.Empty;

        layout.Controls.Add(heading, 0, 0);
        layout.Controls.Add(_latestFrameText, 0, 1);
        layout.Controls.Add(_lastReceivedText, 0, 2);
        layout.Controls.Add(plotHeading, 0, 3);
        layout.Controls.Add(_accelerationPlot, 0, 4);
        return layout;
    }

    private static void ConfigureComboBox(ComboBox comboBox, int width)
    {
        comboBox.Width = width;
        comboBox.Height = 38;
        comboBox.DropDownStyle = ComboBoxStyle.DropDownList;
        comboBox.Font = new Font("Segoe UI", 9.5f);
        comboBox.BackColor = Color.White;
        comboBox.ForeColor = InkColor;
        comboBox.Margin = new Padding(0, 0, 14, 0);
    }

    private static void ConfigurePrimaryButton(Button button, int width)
    {
        button.Width = width;
        button.Height = 38;
        button.Margin = new Padding(0, 26, 0, 0);
        button.FlatStyle = FlatStyle.Flat;
        button.FlatAppearance.BorderSize = 0;
        button.BackColor = AccentColor;
        button.ForeColor = Color.White;
        button.Font = new Font("Segoe UI", 9, FontStyle.Bold);
        button.Cursor = Cursors.Hand;
    }

    private static void ConfigureSecondaryButton(Button button, int width)
    {
        button.Width = width;
        button.Height = 38;
        button.FlatStyle = FlatStyle.Flat;
        button.FlatAppearance.BorderColor = BorderColor;
        button.FlatAppearance.BorderSize = 1;
        button.BackColor = Color.White;
        button.ForeColor = InkColor;
        button.Font = new Font("Segoe UI", 8.5f);
        button.Cursor = Cursors.Hand;
    }

    #endregion

    #region Port COM

    private void RefreshPorts()
    {
        var previousPort = _portComboBox.SelectedItem as string;

        try
        {
            var ports = SerialPort.GetPortNames()
                .OrderBy(port => port, StringComparer.OrdinalIgnoreCase)
                .ToArray();
            _portComboBox.Items.Clear();
            _portComboBox.Items.AddRange(ports);

            if (previousPort is not null && ports.Contains(previousPort, StringComparer.OrdinalIgnoreCase))
            {
                _portComboBox.SelectedItem = ports.First(port =>
                    string.Equals(port, previousPort, StringComparison.OrdinalIgnoreCase));
            }
            else if (ports.Length > 0)
            {
                _portComboBox.SelectedIndex = 0;
            }

            if (_serialPort is null)
            {
                SetDisconnectedStatus(ports.Length == 0
                    ? "Nie znaleziono portów COM"
                    : $"Znaleziono porty: {ports.Length}");
            }
        }
        catch (Exception exception)
        {
            SetDisconnectedStatus("Nie można odczytać portów");
            MessageBox.Show(this, exception.Message, "Błąd portu", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    private void ToggleConnection()
    {
        if (_serialPort?.IsOpen == true)
        {
            DisconnectPort();
            return;
        }

        if (_portComboBox.SelectedItem is not string portName ||
            _baudComboBox.SelectedItem is not string baudText ||
            !int.TryParse(baudText, out var baudRate))
        {
            SetDisconnectedStatus("Wybierz port i prędkość");
            return;
        }

        var port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
        {
            Encoding = Encoding.ASCII,
            Handshake = Handshake.None,
            ReadTimeout = 500
        };
        port.DataReceived += SerialPort_DataReceived;
        _serialPort = port;

        try
        {
            port.Open();
            _portComboBox.Enabled = false;
            _baudComboBox.Enabled = false;
            _refreshButton.Enabled = false;
            _connectButton.Text = "Zamknij port";
            _connectButton.BackColor = Color.FromArgb(171, 75, 64);
            _statusIndicator.ForeColor = Color.FromArgb(81, 202, 157);
            _statusText.Text = "POŁĄCZONO";
            _statusDetail.Text = $"{portName}  ·  {baudRate} baud  ·  8N1";
            SetRangeControlsEnabled(true);
            QueryCurrentRanges();
        }
        catch (Exception exception)
        {
            port.DataReceived -= SerialPort_DataReceived;
            port.Dispose();
            _serialPort = null;
            _rangeCommandPending = false;
            SetRangeControlsEnabled(false);
            SetDisconnectedStatus($"Nie udało się otworzyć {portName}");
            MessageBox.Show(this, exception.Message, "Błąd otwierania portu",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    #endregion

    #region Odbior i prezentacja danych

    // Odbior UART dziala poza watkiem okna; tutaj tylko zbieramy cale linie.
    private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        if (sender is not SerialPort port)
        {
            return;
        }

        try
        {
            var chunk = port.ReadExisting();

            lock (_receiveLock)
            {
                foreach (var character in chunk)
                {
                    if (character == '\n')
                    {
                        var line = _partialLine.ToString().TrimEnd('\r');
                        _partialLine.Clear();
                        if (line.Length > 0)
                        {
                            _pendingFrames.Enqueue(new ReceivedFrame(DateTime.Now, line));
                        }
                    }
                    else if (_partialLine.Length < MaximumPartialLineLength)
                    {
                        _partialLine.Append(character);
                    }
                    else
                    {
                        _partialLine.Clear();
                    }
                }
            }
        }
        catch (InvalidOperationException)
        {
        }
        catch (IOException)
        {
        }
    }

    private void RefreshReceivedFrames()
    {
        // Timer przenosi zebrane linie do kontrolek paczka, zeby nie blokowac okna.
        var frames = new List<ReceivedFrame>();
        lock (_receiveLock)
        {
            while (_pendingFrames.Count > 0)
            {
                frames.Add(_pendingFrames.Dequeue());
            }
        }

        if (frames.Count == 0)
        {
            return;
        }

        foreach (var frame in frames)
        {
            if (TryParseRangeConfirmation(frame.Line, out var accelRange, out var gyroRange))
            {
                SetConfirmedRanges(accelRange, gyroRange);
            }
            else if (frame.Line.StartsWith("RANGE_ERR,", StringComparison.Ordinal))
            {
                _rangeCommandPending = false;
                SetRangeControlsEnabled(_serialPort?.IsOpen == true);
                _statusDetail.Text = $"Zakres niezmieniony: {frame.Line[10..]}";
            }
        }

        var displayedFrames = new string[frames.Count];
        for (var index = 0; index < frames.Count; index++)
        {
            var frame = frames[index];
            if (TryParseMpu6050Frame(frame.Line, out var sensorFrame))
            {
                var (accelXG, accelYG, accelZG) = ConvertAcceleration(
                    sensorFrame.AccelX, sensorFrame.AccelY, sensorFrame.AccelZ);
                _accelerationPlot.AddSample(frame.Timestamp, accelXG, accelYG, accelZG);
            }
            else if (TryParseAccelerometerFrame(frame.Line, out var accelX, out var accelY, out var accelZ))
            {
                var (accelXG, accelYG, accelZG) = ConvertAcceleration(accelX, accelY, accelZ);
                _accelerationPlot.AddSample(frame.Timestamp, accelXG, accelYG, accelZG);
            }

            var displayedFrame = FormatFrame(frame.Line, multiline: false);
            displayedFrames[index] = $"{frame.Timestamp:HH:mm:ss.fff}  {displayedFrame}";
        }

        _receivedListBox.BeginUpdate();
        try
        {
            _receivedListBox.Items.AddRange(displayedFrames);
            while (_receivedListBox.Items.Count > MaximumDisplayedFrames)
            {
                _receivedListBox.Items.RemoveAt(0);
            }

            _receivedListBox.TopIndex = _receivedListBox.Items.Count - 1;
        }
        finally
        {
            _receivedListBox.EndUpdate();
        }

        var latestFrame = frames[^1];
        var latestDisplay = FormatFrame(latestFrame.Line, multiline: true);
        _receivedFrameCount += frames.Count;
        _frameCountText.Text = _receivedFrameCount.ToString("N0");
        _latestFrameText.Text = latestDisplay;
        _lastReceivedText.Text = $"Czas: {latestFrame.Timestamp:HH:mm:ss.fff}";
    }

    private string FormatFrame(string line, bool multiline)
    {
        if (TryParseMpu6050Frame(line, out var frame))
        {
            var (accelXG, accelYG, accelZG) = ConvertAcceleration(frame.AccelX, frame.AccelY, frame.AccelZ);
            var acceleration = $"x={FormatSignedG(accelXG)}, y={FormatSignedG(accelYG)}, z={FormatSignedG(accelZG)}";
            var magnitude = Math.Sqrt(accelXG * accelXG + accelYG * accelYG + accelZG * accelZG)
                .ToString("F3", CultureInfo.CurrentCulture);
            return multiline
                ? $"ACC [g]: {acceleration}\n|a|: {magnitude} g | TEMP: {frame.TemperatureCelsius:F2} °C\nGYRO raw: x={frame.GyroX}, y={frame.GyroY}, z={frame.GyroZ}"
                : $"ACC [g]: {acceleration} | |a|={magnitude} g | TEMP: {frame.TemperatureCelsius:F2} °C | GYRO raw: x={frame.GyroX}, y={frame.GyroY}, z={frame.GyroZ}";
        }

        return TryParseAccelerometerFrame(line, out var x, out var y, out var z)
            ? FormatAccelerometerG(x, y, z)
            : line;
    }

    private string FormatAccelerometerG(int accelX, int accelY, int accelZ)
    {
        var (accelXG, accelYG, accelZG) = ConvertAcceleration(accelX, accelY, accelZ);
        var magnitude = Math.Sqrt(accelXG * accelXG + accelYG * accelYG + accelZG * accelZG);
        return $"ACC [g]: x={FormatSignedG(accelXG)}, y={FormatSignedG(accelYG)}, z={FormatSignedG(accelZG)} | |a|={magnitude:F3} g";
    }

    private (double X, double Y, double Z) ConvertAcceleration(int accelX, int accelY, int accelZ)
    {
        var accelXG = CorrectAcceleration(accelX / _accelerometerCountsPerG, AccelerometerXBiasG, AccelerometerXScale);
        var accelYG = CorrectAcceleration(accelY / _accelerometerCountsPerG, AccelerometerYBiasG, AccelerometerYScale);
        var accelZG = CorrectAcceleration(accelZ / _accelerometerCountsPerG, AccelerometerZBiasG, AccelerometerZScale);
        return (accelXG, accelYG, accelZG);
    }

    private static double CorrectAcceleration(double valueG, double biasG, double scale) =>
        (valueG - biasG) * scale;

    private static string FormatSignedG(double value) =>
        value.ToString("+0.000;-0.000;0.000", CultureInfo.CurrentCulture);

    // Format MPU6050: ACC=x,y,z TEMP=t GYRO=x,y,z. Drugi format to ACC,x,y,z.
    private static bool TryParseMpu6050Frame(string line, out Mpu6050Frame frame)
    {
        frame = default;
        var fields = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (fields.Length != 3 ||
            !fields[0].StartsWith("ACC=", StringComparison.Ordinal) ||
            !fields[1].StartsWith("TEMP=", StringComparison.Ordinal) ||
            !fields[2].StartsWith("GYRO=", StringComparison.Ordinal))
        {
            return false;
        }

        var accelerometer = fields[0][4..].Split(',');
        var gyroscope = fields[2][5..].Split(',');
        if (accelerometer.Length != 3 || gyroscope.Length != 3 ||
            !int.TryParse(accelerometer[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out var accelX) ||
            !int.TryParse(accelerometer[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var accelY) ||
            !int.TryParse(accelerometer[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var accelZ) ||
            !int.TryParse(fields[1].AsSpan(5), NumberStyles.Integer, CultureInfo.InvariantCulture, out var temperatureRaw) ||
            !int.TryParse(gyroscope[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out var gyroX) ||
            !int.TryParse(gyroscope[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var gyroY) ||
            !int.TryParse(gyroscope[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var gyroZ))
        {
            return false;
        }

        frame = new Mpu6050Frame(accelX, accelY, accelZ, temperatureRaw, gyroX, gyroY, gyroZ);
        return true;
    }

    private static bool TryParseAccelerometerFrame(string line, out int x, out int y, out int z)
    {
        x = 0;
        y = 0;
        z = 0;

        var fields = line.Split(',');
        return fields.Length == 4 &&
            string.Equals(fields[0].Trim(), "ACC", StringComparison.Ordinal) &&
            int.TryParse(fields[1].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out x) &&
            int.TryParse(fields[2].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out y) &&
            int.TryParse(fields[3].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out z);
    }

    #endregion

    #region Czyszczenie i zamykanie

    private void ClearReceivedFrames()
    {
        lock (_receiveLock)
        {
            _pendingFrames.Clear();
        }

        _receivedListBox.Items.Clear();
        _receivedFrameCount = 0;
        _frameCountText.Text = "0";
        _latestFrameText.Text = "—";
        _lastReceivedText.Text = "Brak odebranych danych";
        _accelerationPlot.ClearSamples();
    }

    private void DisconnectPort()
    {
        var port = _serialPort;
        _serialPort = null;

        if (port is not null)
        {
            port.DataReceived -= SerialPort_DataReceived;
            try
            {
                if (port.IsOpen)
                {
                    port.Close();
                }
            }
            catch (IOException)
            {
            }
            finally
            {
                port.Dispose();
            }
        }

        lock (_receiveLock)
        {
            _partialLine.Clear();
            _pendingFrames.Clear();
        }

        if (IsDisposed)
        {
            return;
        }

        _rangeCommandPending = false;
        SetRangeControlsEnabled(false);
        _portComboBox.Enabled = true;
        _baudComboBox.Enabled = true;
        _refreshButton.Enabled = true;
        _connectButton.Text = "Otwórz port";
        _connectButton.BackColor = AccentColor;
        SetDisconnectedStatus("Port zamknięty");
    }

    private void SetDisconnectedStatus(string detail)
    {
        _statusIndicator.ForeColor = Color.FromArgb(217, 109, 97);
        _statusText.Text = "ROZŁĄCZONO";
        _statusDetail.Text = detail;
    }

    #endregion
}

internal sealed class AccelerationPlot : Control
{
    #region Ustawienia wykresu

    private static readonly Color GridColor = Color.FromArgb(226, 233, 231);
    private static readonly Color AxisXColor = Color.FromArgb(8, 126, 120);
    private static readonly Color AxisYColor = Color.FromArgb(207, 91, 79);
    private static readonly Color AxisZColor = Color.FromArgb(205, 142, 39);
    private readonly Queue<(DateTime Timestamp, double X, double Y, double Z)> _samples = new();
    private const int MaximumSamples = 360;
    private static readonly TimeSpan HistoryDuration = TimeSpan.FromMinutes(3);
    private double _rangeG = 2.0;

    #endregion

    #region Dane i rysowanie wykresu

    public AccelerationPlot()
    {
        BackColor = Color.White;
        DoubleBuffered = true;
        ResizeRedraw = true;
    }

    public void AddSample(DateTime timestamp, double x, double y, double z)
    {
        _samples.Enqueue((timestamp, x, y, z));
        var cutoff = timestamp - HistoryDuration;
        while (_samples.Count > 0 && _samples.Peek().Timestamp < cutoff)
        {
            _samples.Dequeue();
        }

        while (_samples.Count > MaximumSamples)
        {
            _samples.Dequeue();
        }

        Invalidate();
    }

    public void ClearSamples()
    {
        _samples.Clear();
        Invalidate();
    }

    public void SetRange(double fullScaleG)
    {
        if (fullScaleG <= 0.0)
        {
            return;
        }

        _rangeG = fullScaleG;
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        var graphics = e.Graphics;
        graphics.SmoothingMode = SmoothingMode.AntiAlias;
        graphics.Clear(BackColor);

        var plot = new RectangleF(46, 30, ClientSize.Width - 58, ClientSize.Height - 62);
        if (plot.Width <= 20 || plot.Height <= 20)
        {
            return;
        }

        DrawLegend(graphics);
        DrawGrid(graphics, plot, _rangeG);

        if (_samples.Count == 0)
        {
            using var emptyFont = new Font("Segoe UI", 9);
            using var emptyBrush = new SolidBrush(Color.FromArgb(112, 129, 127));
            const string emptyText = "Oczekiwanie na próbki z MPU-6050";
            var textSize = graphics.MeasureString(emptyText, emptyFont);
            graphics.DrawString(emptyText, emptyFont, emptyBrush,
                plot.Left + (plot.Width - textSize.Width) / 2,
                plot.Top + (plot.Height - textSize.Height) / 2);
            return;
        }

        var samples = _samples.ToArray();
        DrawSeries(graphics, plot, samples, sample => sample.X, AxisXColor, _rangeG);
        DrawSeries(graphics, plot, samples, sample => sample.Y, AxisYColor, _rangeG);
        DrawSeries(graphics, plot, samples, sample => sample.Z, AxisZColor, _rangeG);
        DrawTimeLabels(graphics, plot, samples);
    }

    private static void DrawLegend(Graphics graphics)
    {
        using var font = new Font("Segoe UI", 8.5f, FontStyle.Bold);
        using var xPen = new Pen(AxisXColor, 2.5f);
        using var yPen = new Pen(AxisYColor, 2.5f);
        using var zPen = new Pen(AxisZColor, 2.5f);
        using var brush = new SolidBrush(Color.FromArgb(29, 48, 50));
        DrawEntry("X", xPen, 46);
        DrawEntry("Y", yPen, 96);
        DrawEntry("Z", zPen, 146);

        void DrawEntry(string text, Pen pen, float left)
        {
            graphics.DrawLine(pen, left, 13, left + 15, 13);
            graphics.DrawString(text, font, brush, left + 20, 5);
        }
    }

    private static void DrawGrid(Graphics graphics, RectangleF plot, double rangeG)
    {
        using var gridPen = new Pen(GridColor, 1);
        using var referencePen = new Pen(Color.FromArgb(153, 173, 169), 1)
        {
            DashStyle = DashStyle.Dash
        };
        using var labelFont = new Font("Segoe UI", 8);
        using var labelBrush = new SolidBrush(Color.FromArgb(112, 129, 127));

        var gridStep = rangeG / 4.0;
        for (var value = -rangeG; value <= rangeG + gridStep * 0.001; value += gridStep)
        {
            var y = ValueToY(value, plot, rangeG);
            var isGravityReference = Math.Abs(Math.Abs(value) - 1.0) < 0.001;
            graphics.DrawLine(isGravityReference ? referencePen : gridPen, plot.Left, y, plot.Right, y);
            graphics.DrawString(value.ToString("F1", CultureInfo.CurrentCulture),
                labelFont, labelBrush, 2, y - 8);
        }

        graphics.DrawLine(gridPen, plot.Left, plot.Top, plot.Left, plot.Bottom);
        graphics.DrawLine(gridPen, plot.Left, plot.Bottom, plot.Right, plot.Bottom);
    }

    private static void DrawSeries(
        Graphics graphics,
        RectangleF plot,
        (DateTime Timestamp, double X, double Y, double Z)[] samples,
        Func<(DateTime Timestamp, double X, double Y, double Z), double> selector,
        Color color,
        double rangeG)
    {
        var firstTimestamp = samples[0].Timestamp;
        var duration = (samples[^1].Timestamp - firstTimestamp).TotalSeconds;
        var points = new PointF[samples.Length];
        for (var index = 0; index < samples.Length; index++)
        {
            var sample = samples[index];
            var x = duration <= 0
                ? plot.Right
                : plot.Left + (float)((sample.Timestamp - firstTimestamp).TotalSeconds / duration) * plot.Width;
            points[index] = new PointF(x, ValueToY(selector(sample), plot, rangeG));
        }

        var graphicsState = graphics.Save();
        graphics.SetClip(plot);
        using var pen = new Pen(color, 2);
        using var brush = new SolidBrush(color);
        if (points.Length > 1)
        {
            graphics.DrawLines(pen, points);
        }
        else
        {
            graphics.FillEllipse(brush, points[0].X - 3, points[0].Y - 3, 6, 6);
        }

        graphics.Restore(graphicsState);
    }

    private static float ValueToY(double value, RectangleF plot, double rangeG)
    {
        var clampedValue = Math.Clamp(value, -rangeG, rangeG);
        return plot.Bottom - (float)((clampedValue + rangeG) / (2.0 * rangeG)) * plot.Height;
    }

    private static void DrawTimeLabels(
        Graphics graphics,
        RectangleF plot,
        (DateTime Timestamp, double X, double Y, double Z)[] samples)
    {
        using var font = new Font("Segoe UI", 8);
        using var brush = new SolidBrush(Color.FromArgb(112, 129, 127));
        graphics.DrawString(samples[0].Timestamp.ToString("HH:mm:ss"), font, brush,
            plot.Left, plot.Bottom + 5);
        var lastLabel = samples[^1].Timestamp.ToString("HH:mm:ss");
        var lastSize = graphics.MeasureString(lastLabel, font);
        graphics.DrawString(lastLabel, font, brush, plot.Right - lastSize.Width, plot.Bottom + 5);
    }

    #endregion
}