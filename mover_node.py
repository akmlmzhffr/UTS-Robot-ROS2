#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Penggerak otomatis untuk TRACK KIRI UTS (Gazebo Harmonic).

Rute ini dibuat sebagai pencerminan kiri-kanan dari mover_track_kanan.py
milik teman: posisi awal di titik biru tengah-bawah, menghadap ke ATAS pada
pandangan atas Gazebo. Robot mulai otomatis; tidak perlu teleop di gerbang start.

Batas ROS yang digunakan saat runtime:
  - publish /cmd_vel (geometry_msgs/msg/Twist)
  - subscribe /wheel/speed (std_msgs/msg/Float64MultiArray), untuk pemantauan saja

Tidak membaca /clock, /odom, TF, world SDF, mesh OBJ, atau API internal Gazebo.
Gerak bersifat open-loop berbasis durasi, sama dengan file teman; waktu selesai
rute bukan sensor garis finish. Periksa secara visual ketika menguji simulasi.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Disamakan dengan file teman yang berhasil pada track kanan.
KECEPATAN_LINEAR = 3.0               # m/s
KECEPATAN_SUDUT = math.radians(180)  # rad/s (pi rad/s = 180 derajat/detik)
PERIODE_TIMER = 0.02                 # detik, 50 Hz


def maju(jarak, ket=''):
    return ('maju', jarak, ket)


def putar_kiri(derajat, ket=''):
    return ('putar', +derajat, ket)


def putar_kanan(derajat, ket=''):
    return ('putar', -derajat, ket)


def busur_kiri(radius, derajat, ket=''):
    return ('busur', (radius, +derajat), ket)


def busur_kanan(radius, derajat, ket=''):
    return ('busur', (radius, -derajat), ket)


def diam(detik, ket=''):
    return ('diam', detik, ket)


# TRACK KIRI: pencerminan kiri-kanan urutan track kanan dari teman.
# Asumsi awal tetap seperti file sumber: titik biru tengah-bawah, menghadap atas.
RUTE = [
    diam(1.0,                 'Tunggu koneksi ke /cmd_vel'),

    # Dari titik start menuju mulut lorong kiri.
    putar_kiri(87,            'Hadap kiri'),
    maju(8.40,                'Maju ke depan mulut lorong kiri'),
    putar_kanan(120,          'Hadap atas, siap masuk lorong'),

    # Masuk lorong dan bergeser ke kiri.
    maju(8.13,                'Masuk lorong sampai bagian yang melebar'),
    putar_kiri(117,           'Hadap kiri'),
    maju(1.90,                'Geser ke kiri'),
    putar_kanan(115,          'Hadap atas'),
    maju(3.80,                'Naik sampai ujung lorong sempit'),

    # Lorong ke kanan, kemudian naik sepanjang sisi kanan.
    putar_kanan(113,          'Hadap kanan'),
    maju(7.00,                'Maju ke kanan sampai pojok'),
    putar_kiri(115,           'Hadap atas'),
    maju(14.00,               'Naik sepanjang lorong kanan'),

    # Lorong atas dan setengah lingkaran yang dicerminkan ke kiri.
    putar_kiri(115,           'Hadap kiri'),
    maju(16.00,               'Maju sepanjang lorong atas'),
    busur_kiri(4.50, 180,     'Tikungan setengah lingkaran (U-turn ke kiri)'),
    diam(1.0,                 'Jeda setelah setengah lingkaran'),
    putar_kiri(8,             'Penyesuaian arah setelah U-turn'),

    # Kembali ke kanan lalu zig-zag menuju ruangan.
    maju(6.70,                'Maju ke kanan di bawah tikungan'),
    putar_kanan(120,          'Hadap bawah'),
    maju(3.50,                'Turun'),
    putar_kanan(117,          'Hadap kiri'),
    maju(8.25,                'Maju ke kiri sampai di atas bukaan ruangan'),
    putar_kiri(115,           'Hadap bawah'),
    maju(5.54,                'Masuk ruangan'),

    # Dari ruangan ke kanal sempit kiri dan lorong FINISH.
    putar_kanan(115,          'Hadap kiri'),
    maju(2.62,                'Geser ke kiri, tepat di atas kanal sempit'),
    putar_kiri(117,           'Hadap bawah'),
    maju(6.70,                'Turun melalui kanal sempit sampai lorong bawah'),
    putar_kiri(120,           'Hadap kanan'),
    maju(7.38,                'Maju ke kanan sampai tulisan FINISH, lalu berhenti'),
]


def susun_langkah(rute):
    """Konversi rute menjadi (keterangan, linear.x, angular.z, durasi)."""
    langkah = []
    for jenis, nilai, ket in rute:
        if jenis == 'maju':
            if nilai < 0:
                raise ValueError('Jarak maju harus tidak negatif.')
            langkah.append((ket, KECEPATAN_LINEAR, 0.0, nilai / KECEPATAN_LINEAR))
        elif jenis == 'putar':
            arah = 1.0 if nilai > 0 else -1.0
            durasi = math.radians(abs(nilai)) / KECEPATAN_SUDUT
            langkah.append((ket, 0.0, arah * KECEPATAN_SUDUT, durasi))
        elif jenis == 'busur':
            radius, derajat = nilai
            if radius <= 0:
                raise ValueError('Radius busur harus lebih besar dari nol.')
            arah = 1.0 if derajat > 0 else -1.0
            omega = KECEPATAN_LINEAR / radius
            durasi = math.radians(abs(derajat)) / omega
            langkah.append((ket, KECEPATAN_LINEAR, arah * omega, durasi))
        elif jenis == 'diam':
            langkah.append((ket, 0.0, 0.0, nilai))
        else:
            raise ValueError(f'Jenis langkah tidak dikenal: {jenis}')
    return langkah


class MoverTrackKiri(Node):
    def __init__(self):
        super().__init__('mover_track_kiri')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheel_sub = self.create_subscription(
            Float64MultiArray, '/wheel/speed', self.wheel_callback, 10
        )
        self.latest_left = None
        self.latest_right = None
        self.last_wheel_log = 0.0

        self.langkah = susun_langkah(RUTE)
        self.waktu_selesai = []
        total = 0.0
        for _, _, _, durasi in self.langkah:
            total += durasi
            self.waktu_selesai.append(total)
        self.total_rute = total

        self.indeks = -1
        self.waktu_mulai = None
        self.waktu_mulai_wall = None
        self.selesai = False
        self.timer = self.create_timer(PERIODE_TIMER, self.timer_callback)
        self.get_logger().info(
            f'{len(self.langkah)} langkah; estimasi durasi rute {self.total_rute:.2f} detik.'
        )

    def berhenti(self):
        self.publisher_.publish(Twist())

    def wheel_callback(self, msg):
        if len(msg.data) >= 2:
            self.latest_left = float(msg.data[0])
            self.latest_right = float(msg.data[1])

    def timer_callback(self):
        now = time.monotonic()
        if self.waktu_mulai_wall is None:
            self.waktu_mulai_wall = now
            self.get_logger().info(
                'Mulai otomatis dari titik tengah-bawah; diasumsikan robot menghadap ATAS.'
            )
        elapsed = now - self.waktu_mulai_wall

        i = max(self.indeks, 0)
        while i < len(self.langkah) and elapsed >= self.waktu_selesai[i]:
            i += 1

        if i >= len(self.langkah):
            self.berhenti()
            self.timer.cancel()
            self.selesai = True
            self.get_logger().info('Urutan rute selesai; perintah berhenti dikirim.')
            print(f'Waktu eksekusi rute: {elapsed:.2f} detik', flush=True)
            print('Periksa secara visual apakah robot telah mencapai garis FINISH.', flush=True)
            return

        if i != self.indeks:
            self.indeks = i
            self.get_logger().info(f'[{i + 1}/{len(self.langkah)}] {self.langkah[i][0]}')

        _, v, w, _ = self.langkah[i]
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

        if elapsed - self.last_wheel_log >= 1.0:
            self.last_wheel_log = elapsed
            if self.latest_left is None:
                wheel_text = '/wheel/speed belum menerima data'
            else:
                wheel_text = (
                    f'roda kiri={self.latest_left:.2f} rad/s, '
                    f'kanan={self.latest_right:.2f} rad/s'
                )
            self.get_logger().info(f't={elapsed:.1f}/{self.total_rute:.1f} s | {wheel_text}')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MoverTrackKiri()
    try:
        print('\n=== MOVER NODE: TRACK KIRI ===')
        print('Rute dicerminkan dari mover_track_kanan.py teman Anda.')
        print('Topik runtime hanya /cmd_vel (publish) dan /wheel/speed (monitor).')
        print('Asumsi awal: titik biru tengah-bawah dan robot menghadap ATAS.')
        print('Gerakan langsung mulai otomatis; tidak perlu teleop untuk mengarahkan ke gerbang.\n')
        while rclpy.ok() and not node.selesai:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Dihentikan manual (Ctrl+C).')
    finally:
        for _ in range(10):
            node.berhenti()
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
