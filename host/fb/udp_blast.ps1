# udp_blast.ps1 - native Windows line-rate receive test for board A (no Python needed).
#   powershell -File host\fb\udp_blast.ps1 [-Count 200000] [-Size 1472] [-Mbps 950]
# Sends paced unsequenced junk datagrams (type 0x7F, seq 0) that board A counts and discards, and reads
# the board's counters (STATUS) before and after: every datagram should show up in rx_ok / udp with
# fcs, pfdrop and ovf at 0. Pacing matters: Windows silently drops UDP when the NIC queue is full.
# Measured: 1 000 000 x 1472 B at 990 Mbit/s (118 MB/s) and 100 000 at 99 Mbit/s, all received.
param([int]$Count = 200000, [int]$Size = 1472, [double]$Mbps = 950)
Add-Type -TypeDefinition @"
using System; using System.Net; using System.Net.Sockets; using System.Diagnostics;
public static class FbBlast {
    static byte[] Hdr(byte t) { var b = new byte[8]; b[0]=0x46; b[1]=0x42; b[2]=1; b[3]=t; return b; }
    public static uint[] Status(UdpClient u, IPEndPoint ep) {
        var req = Hdr(0x02);
        for (int tries = 0; tries < 20; tries++) {
            u.Send(req, req.Length, ep);
            var sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < 200) {
                if (u.Available > 0) {
                    IPEndPoint from = null; var d = u.Receive(ref from);
                    if (d.Length >= 8 + 160 && d[3] == 0x82) {
                        var w = new uint[40]; for (int i = 0; i < 40; i++) w[i] = BitConverter.ToUInt32(d, 8 + 4*i);
                        return w;
                    }
                } else System.Threading.Thread.Sleep(1);
            }
        }
        return null;
    }
    public static string Run(int count, int size, double wireMbps) {
        double tick = (size + 8 + 20 + 14 + 4 + 8 + 12) * 8.0 / (wireMbps * 1e6);   // seconds per frame on the wire
        var ep = new IPEndPoint(IPAddress.Parse("192.168.1.10"), 5000);
        var u = new UdpClient(new IPEndPoint(IPAddress.Parse("192.168.1.112"), 0));
        u.Client.SendBufferSize = 8 << 20; u.Client.ReceiveBufferSize = 8 << 20;
        var s0 = Status(u, ep); System.Threading.Thread.Sleep(20); while (u.Available > 0) { IPEndPoint f = null; u.Receive(ref f); }
        s0 = Status(u, ep);
        var p = new byte[size]; var h = Hdr(0x7F); Array.Copy(h, p, 8);
        for (int i = 8; i < size; i++) p[i] = (byte)i;
        var sw = Stopwatch.StartNew();
        double freq = Stopwatch.Frequency;
        for (int i = 0; i < count; i++) {
            while (sw.ElapsedTicks / freq < i * tick) { }
            u.Send(p, p.Length, ep);
        }
        double secs = sw.Elapsed.TotalSeconds;
        System.Threading.Thread.Sleep(300);                       // let the board finish; then drop the periodic
        while (u.Available > 0) { IPEndPoint f = null; u.Receive(ref f); }   // STATUS datagrams queued during the blast
        var s1 = Status(u, ep);
        u.Close();
        if (s0 == null || s1 == null) return "no STATUS";
        double mbps = (double)count * (size + 8 + 20 + 14 + 4 + 8 + 12) * 8 / secs / 1e6;
        return String.Format("sent {0} x {1} B in {2:F3} s = {3:F1} MB/s payload, {4:F0} Mbit/s on the wire; board: rx_ok +{5} fcs +{6} udp +{7} pfdrop +{8} ovf +{9}",
            count, size, secs, count * (double)size / secs / 1e6, mbps,
            s1[5] - s0[5], s1[6] - s0[6], s1[12] - s0[12], s1[15] - s0[15], s1[8] - s0[8]);
    }
}
"@
[FbBlast]::Run($Count, $Size, $Mbps)
