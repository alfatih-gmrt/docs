# Command Line Interface

Command Line Interface (CLI) adalah antarmuka pengguna berbasis teks yang memungkinkan kita berinteraksi dengan komputer melalui perintah tertulis.

Dengan CLI, pengguna mengetikkan **command**(perintah) di terminal untuk menjalankan program.

## Shell

Shell adalah aplikasi yang menjadi antarmuka untuk mengetikkan **command**.

Contoh-contoh shell:
- Command Prompt (Windows)
- PowerShell (Windows)
- Bash (Bourne Again Shell)
- Zsh (Z Shell)

### Bash

Bash adalah shell yang digunakan secara default oleh Ubuntu. Meskipun demikian, jenis shell lain juga bisa digunakan di Ubuntu, seperti Zsh.

### Contoh Command di Linux/Ubuntu

#### Navigasi Direktori

```bash
pwd     # present working directory
```
Menampilkan direktori saat ini.

```bash
ls
ls -l   # long listing format
ls -a   # all
```
Melihat isi direktori

```bash
cd directory_name   # masuk ke dalam direktori
cd ..               # naik 1 level direktori
cd ~                # kembali ke home
```

#### File & Directory

```bash
touch file.txt          # membuat file kosong

mkdir folder            # membuat direktori baru
mkdir -p a/b/c          # membuat direktori bertingkat

cp main.cpp hehe.cpp    # copy file dari main.cpp ke hehe.cpp
cp -r dir1 dir2         # copy dir1 ke dir2

mv file1.txt file2.txt  # rename / pindah

rm file.txt             # hapus file
rm -r folder            # hapus folder
```

#### Melihat isi file

```bash
cat file.txt    # menampilkan isi file
```

#### Environment & Shell
```bash
echo $ROS_DOMAIN_ID     # menampilkan environment variable
export $ROS_DOMAIN_ID   # set env var untuk session saat ini

which python3           # menampilkan lokasi command

history                 # menampilkan history command
clear                   # membersihkan terminal
```

## Sudo (Superuser Do)

User biasa hanya diberi akses secara bebas pada direktori ```/home/user```(home). Selain itu, direktori lain, seperti ```/```(root), ```/opt```, dan ```/usr``` tidak boleh sembarangan diakses karena bisa mengubah konfigurasi sistem.

Menambahkan kata ```sudo``` sebelum command dapat memberikan akses untuk mengubah **root**.  
Contoh:
```bash
sudo apt install terminator
```
Jika command tersebut dijalankan, terminator akan diinstall di ```/usr/bin```.

## Penting!

Setiap **command** memiliki penjelasan untuk setiap command dan option-nya. Bisa ditampilkan dengan mengetikkan option/flag ```-h``` atau ```--help```.  
Contoh:
```bash
ls -h       # short option/flag
ls --help   # long option/flag
```

*Environment Variable* adalah variabel sistem yang digunakan untuk menyimpan konfigurasi dan mempengaruhi perilaku program yang berjalan di dalam suatu environment (shell / OS).  
Contoh:
```bash
echo $HOME                  # menampilkan env var HOME
export $ROS_DOMAIN_ID=69    # mengatur nilai env var ROS_DOMAIN_ID 
```

Ada beberapa alias yang sering digunakan di Linux.  
```.``` = direktori saat ini.  
```..``` = direktori di atasnya.  
```~``` = direktori home, exp: ```/home/user```