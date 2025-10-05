import network, socket, json, time, math
from machine import Pin, I2C, PWM

# ====== CONFIG ======
WIFI_SSID = "VONETS_2.4G_B568"
WIFI_PASS = "12345678"
UDP_PORT  = 9000

ESC_FREQ_HZ = 50        # 50 Hz muy compatible (sube a 400 si tus ESC lo soportan)
MIN_US = 1000
MAX_US = 2000
ARM_US = 1000

MOTOR_PINS = [2, 3, 4, 5]  # GP2..GP5

I2C_ID = 0; SDA_PIN = 0; SCL_PIN = 1
IMU_ADDR = 0x68

# Ganancias (empieza suave; ajusta en pruebas)
Kp_roll  = 2.0; Kd_roll  = 0.04
Kp_pitch = 2.0; Kd_pitch = 0.04
Kp_yaw   = 1.0; Kd_yaw   = 0.01
Kp_z = 2.0; Kd_z = 0.8

T_base = 0.40           # thrust medio (ajústalo a tu frame)
PKT_TIMEOUT_S = 0.5     # failsafe (no tocar: lo pediste igual)

# ====== PWM utils ======
def setup_esc_pwm(pins, freq_hz):
    pwms = []
    for gp in pins:
        p = PWM(Pin(gp)); p.freq(freq_hz); p.duty_u16(0); pwms.append(p)
    return pwms

def us_to_duty(us, freq_hz):
    period_us = 1_000_000 // freq_hz
    us = max(MIN_US, min(MAX_US, us))
    return int(us * 65535 // period_us)

def write_esc_microseconds(pwms, us_list):
    for p, us in zip(pwms, us_list):
        p.duty_u16(us_to_duty(us, p.freq()))

def disarm(pwms):
    write_esc_microseconds(pwms, [ARM_US]*4)

def arm_escs(pwms, t_ms=1500):
    end = time.ticks_add(time.ticks_ms(), t_ms)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        disarm(pwms); time.sleep_ms(20)

# ====== IMU (MPU6050 min) + Mahony ======
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c=i2c; self.addr=addr
        self.i2c.writeto_mem(addr, 0x6B, b'\x00') # wake
        self.i2c.writeto_mem(addr, 0x1B, b'\x18') # gyro ±2000 dps
        self.i2c.writeto_mem(addr, 0x1C, b'\x08') # acc ±4g
        self.i2c.writeto_mem(addr, 0x1A, b'\x03') # DLPF
        self.acc_scale = (4*9.80665)/32768.0
        self.gyr_scale = 2000.0/32768.0
        self.ab=[0,0,0]; self.gb=[0,0,0]
    def _i16(self,h,l):
        v=(h<<8)|l; return v-65536 if v>32767 else v
    def raw(self):
        d=self.i2c.readfrom_mem(self.addr,0x3B,14)
        ax=self._i16(d[0],d[1]); ay=self._i16(d[2],d[3]); az=self._i16(d[4],d[5])
        gx=self._i16(d[8],d[9]); gy=self._i16(d[10],d[11]); gz=self._i16(d[12],d[13])
        return ax,ay,az,gx,gy,gz
    def read(self):
        ax,ay,az,gx,gy,gz=self.raw()
        ax=ax*self.acc_scale - self.ab[0]; ay=ay*self.acc_scale - self.ab[1]; az=az*self.acc_scale - self.ab[2]
        gx=gx*self.gyr_scale - self.gb[0]; gy=gy*self.gyr_scale - self.gb[1]; gz=gz*self.gyr_scale - self.gb[2]
        return ax,ay,az,gx,gy,gz
    def calibrate(self,n=600):
        print("Calibrando IMU... no mover")
        sa=[0,0,0]; sg=[0,0,0]
        for _ in range(n):
            ax,ay,az,gx,gy,gz=self.raw()
            sa[0]+=ax; sa[1]+=ay; sa[2]+=az; sg[0]+=gx; sg[1]+=gy; sg[2]+=gz
            time.sleep_ms(2)
        sa=[s/n for s in sa]; sg=[s/n for s in sg]
        self.ab=[sa[0]*self.acc_scale, sa[1]*self.acc_scale, sa[2]*self.acc_scale-9.80665]
        self.gb=[sg[0]*self.gyr_scale, sg[1]*self.gyr_scale, sg[2]*self.gyr_scale]
        print("Bias acc:", self.ab, "Bias gyro:", self.gb)

class Mahony:
    def __init__(self,Kp=3.0,Ki=0.02):
        self.Kp=Kp; self.Ki=Ki; self.q=[1,0,0,0]; self.ei=[0,0,0]
    def update(self,gx,gy,gz,ax,ay,az,dt):
        gx*=math.pi/180; gy*=math.pi/180; gz*=math.pi/180
        q0,q1,q2,q3=self.q
        n=math.sqrt(ax*ax+ay*ay+az*az)
        if n>1e-6:
            ax/=n; ay/=n; az/=n
            vx=2*(q1*q3 - q0*q2); vy=2*(q0*q1 + q2*q3); vz=q0*q0 - q1*q1 - q2*q2 + q3*q3
            ex=(ay*vz-az*vy); ey=(az*vx-ax*vz); ez=(ax*vy-ay*vx)
            self.ei[0]+=self.Ki*ex*dt; self.ei[1]+=self.Ki*ey*dt; self.ei[2]+=self.Ki*ez*dt
            gx+=self.Kp*ex+self.ei[0]; gy+=self.Kp*ey+self.ei[1]; gz+=self.Kp*ez+self.ei[2]
        qDot0=0.5*(-q1*gx - q2*gy - q3*gz)
        qDot1=0.5*( q0*gx + q2*gz - q3*gy)
        qDot2=0.5*( q0*gy - q1*gz + q3*gx)
        qDot3=0.5*( q0*gz + q1*gy - q2*gx)
        q0+=qDot0*dt; q1+=qDot1*dt; q2+=qDot2*dt; q3+=qDot3*dt
        inv=1.0/math.sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
        self.q=[q0*inv,q1*inv,q2*inv,q3*inv]
    def euler(self):
        q0,q1,q2,q3=self.q
        roll  = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
        pitch = math.asin( max(-1,min(1,2*(q0*q2 - q3*q1))) )
        yaw   = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        return roll,pitch,yaw

# ====== WiFi / UDP ======
def wifi_connect(ssid, pwd):
    wlan=network.WLAN(network.STA_IF); wlan.active(True); wlan.connect(ssid,pwd)
    t0=time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(),t0)>10000: raise RuntimeError("WiFi timeout")
        time.sleep_ms(200)
    print("WiFi:", wlan.ifconfig()); return wlan

def udp_listener(port):
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0",port)); s.setblocking(False); return s

# ====== Main ======
def main():
    wifi_connect(WIFI_SSID, WIFI_PASS)
    udp = udp_listener(UDP_PORT)
    pwms = setup_esc_pwm(MOTOR_PINS, ESC_FREQ_HZ)
    disarm(pwms); arm_escs(pwms,1500)

    i2c=I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000)
    imu=MPU6050(i2c, IMU_ADDR); time.sleep_ms(100); imu.calibrate(600)
    ahrs=Mahony(3.0,0.02)

    last = time.ticks_us()
    last_pkt_us = time.ticks_us()
    armed_cmd=False
    z=0.0; vz=0.0; yaw_meas=0.0
    z_ref=0.3; yaw_ref=0.0
    prev_roll=prev_pitch=prev_yaw=0.0

    while True:
        now=time.ticks_us()
        dt=max(1e-3, time.ticks_diff(now,last)/1_000_000.0)
        last=now

        # read UDP (no bloquea)
        try:
            data,_=udp.recvfrom(512)
            pkt=json.loads(data)
            if "armed" in pkt: armed_cmd=bool(pkt["armed"])
            if "z" in pkt:   z=float(pkt["z"])
            if "vz" in pkt:  vz=float(pkt["vz"])
            if "yaw" in pkt: yaw_meas=float(pkt["yaw"])
            if "z_ref" in pkt:   z_ref=float(pkt["z_ref"])
            if "yaw_ref" in pkt: yaw_ref=float(pkt["yaw_ref"])
            last_pkt_us=now
        except Exception:
            pass

        # failsafe
        if time.ticks_diff(now,last_pkt_us) > int(PKT_TIMEOUT_S*1_000_000):
            armed_cmd=False

        # IMU attitude
        ax,ay,az,gx,gy,gz=imu.read()
        ahrs.update(gx,gy,gz, ax,ay,az, dt)
        roll,pitch,yaw_imu=ahrs.euler()
        yaw = yaw_meas if time.ticks_diff(now,last_pkt_us)<300_000 else yaw_imu

        # errors
        e_roll = 0.0 - roll
        e_pitch= 0.0 - pitch
        e_yaw  = yaw_ref - yaw
        while e_yaw> math.pi: e_yaw-=2*math.pi
        while e_yaw<-math.pi: e_yaw+=2*math.pi
        e_z = z_ref - z
        ev_z= 0.0 - vz

        d_roll =(roll -prev_roll)/dt; d_pitch=(pitch-prev_pitch)/dt; d_yaw=(yaw-prev_yaw)/dt
        prev_roll=roll; prev_pitch=pitch; prev_yaw=yaw

        u_roll  = Kp_roll*e_roll  - Kd_roll*d_roll
        u_pitch = Kp_pitch*e_pitch- Kd_pitch*d_pitch
        u_yaw   = Kp_yaw*e_yaw    - Kd_yaw*d_yaw

        u_z = Kp_z*e_z + Kd_z*ev_z
        T = T_base + u_z
        T = max(0.1, min(0.9, T))

        # mixer Quad X (m1:FL, m2:FR, m3:RR, m4:RL)
        m1 = T + u_pitch + u_roll + u_yaw
        m2 = T + u_pitch - u_roll - u_yaw
        m3 = T - u_pitch - u_roll + u_yaw
        m4 = T - u_pitch + u_roll - u_yaw

        motors=[m1,m2,m3,m4]
        maxabs=max(1.0, max(abs(x) for x in motors))
        motors=[max(0.0, min(1.0, x/maxabs)) for x in motors]
        us=[MIN_US + int(m*(MAX_US-MIN_US)) for m in motors]

        if armed_cmd: write_esc_microseconds(pwms, us)
        else: disarm(pwms)

        time.sleep(0.004)  # ~200 Hz control loop

try:
    main()
except KeyboardInterrupt:
    pass