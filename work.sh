cd audio_ip_adf/fire_safe_esp32_sdk
. ./export.sh
export ADF_PATH=/home/ubuntu/BoTroGiang/audio_ip_adf/
cd ../../slave
idf.py build | grep -E --color 'error|'
# printenv ADF_PATH




