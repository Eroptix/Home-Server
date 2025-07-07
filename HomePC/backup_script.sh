#!/bin/bash

# === Configuration ===
BACKUP_DIR="/mnt/ssd/backups"
DEST_REMOTE="gdrive:my_backups"
TIMESTAMP=$(date +"%Y-%m-%d")
DAYS_TO_KEEP=7
SW_VERSION="1.0.0"

# Log setup
LOG_DIR="${BACKUP_DIR}/logs"
LOG_FILE="${LOG_DIR}/backup_${TIMESTAMP}.log"
mkdir -p "$LOG_DIR"

log() {
  echo "$(date '+%Y-%m-%d %H:%M:%S') - $*" | tee -a "$LOG_FILE"
}

# Folders to back up
FOLDERS=(
  adguard bitwarden homarr nginx bazarr jellyfin prowlarr python_venv
  qbittorrent sonarr radarr lidarr homepage pigallery homeassistant
  piwol stash whisparr
)

log "========== Starting backup at $TIMESTAMP =========="
log "Current SW Version: ${SW_VERSION}"

# === Backup system configuration ===
log "Backing up system configuration"
SYS_BACKUP_DIR="${BACKUP_DIR}/system_config_${TIMESTAMP}"
mkdir -p "$SYS_BACKUP_DIR"

log "Saving crontab"
crontab -l > "${SYS_BACKUP_DIR}/crontab.txt" 2>/dev/null

log "Saving installed package list"
dpkg --get-selections > "${SYS_BACKUP_DIR}/installed_packages.txt"

log "Backing up systemd services"
cp -r /etc/systemd/system "${SYS_BACKUP_DIR}/systemd_services"

log "Backing up key config files"
cp /etc/fstab "${SYS_BACKUP_DIR}/fstab.txt" 2>/dev/null
cp /etc/hostname "${SYS_BACKUP_DIR}/hostname.txt" 2>/dev/null
cp /etc/hosts "${SYS_BACKUP_DIR}/hosts.txt" 2>/dev/null
cp /etc/network/interfaces "${SYS_BACKUP_DIR}/interfaces.txt" 2>/dev/null || true

# === Backup Docker container info ===
log "Saving Docker container information"
docker ps -a > "${SYS_BACKUP_DIR}/docker_containers.txt"
docker images > "${SYS_BACKUP_DIR}/docker_images.txt"
docker volume ls > "${SYS_BACKUP_DIR}/docker_volumes.txt"

# === Network snapshot ===
log "Saving network configuration"
ip a > "${SYS_BACKUP_DIR}/network_interfaces.txt"
ip route > "${SYS_BACKUP_DIR}/routes.txt"

# === Compress system config backup ===
SYS_CONFIG_ZIP="backup_system_config_${TIMESTAMP}.zip"
zip -r "${BACKUP_DIR}/${SYS_CONFIG_ZIP}" "$SYS_BACKUP_DIR" >/dev/null
rm -rf "$SYS_BACKUP_DIR"
log "System configuration backed up to ${SYS_CONFIG_ZIP}"

# === Compress folders ===
log "Compressing folders"
for folder in "${FOLDERS[@]}"; do
  ZIP_NAME="backup_${folder}_${TIMESTAMP}.zip"
  sudo zip -r "${BACKUP_DIR}/${ZIP_NAME}" "/home/pi/${folder}" >/dev/null
  log "    $folder backed up to $ZIP_NAME"
done

# === Upload to Google Drive ===
log "Uploading backups to Google Drive"
rclone copy "$BACKUP_DIR" "$DEST_REMOTE" --include "*${TIMESTAMP}.zip" --progress

# === Cleanup old backups ===
log "Deleting old remote backups (older than ${DAYS_TO_KEEP} days)"
rclone delete --min-age ${DAYS_TO_KEEP}d "$DEST_REMOTE" --drive-use-trash=false

log "Deleting old local backups (older than ${DAYS_TO_KEEP} days)"
find "$BACKUP_DIR" -name "*.zip" -type f -mtime +${DAYS_TO_KEEP} -delete

# === Google Drive space info ===
log "Checking Google Drive usage:"
rclone about gdrive: | tee -a "$LOG_FILE"

# === Backup manifest summary ===
MANIFEST_FILE="${BACKUP_DIR}/manifest_${TIMESTAMP}.txt"
log "Generating backup manifest: $MANIFEST_FILE"
{
  echo "========== Backup Manifest =========="
  echo "Date: ${TIMESTAMP}"
  echo "Software Version: ${SW_VERSION}"
  echo ""
  echo "Backed up folders:"
  for folder in "${FOLDERS[@]}"; do echo "- ${folder}"; done
  echo ""
  echo "System config archive: ${SYS_CONFIG_ZIP}"
  echo "Google Drive Target: ${DEST_REMOTE}"
} > "$MANIFEST_FILE"
log "Backup manifest written"

log "========== Backup finished =========="
