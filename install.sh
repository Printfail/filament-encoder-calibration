#!/bin/bash
#####################################################################
# ENCODER CALIBRATION + HALL-SENSOR INSTALLATION SCRIPT
# Menü-basiert (Install / Update / Deinstall / Status)
#####################################################################

VERSION="1.0.0"
GITHUB_REPO="https://github.com/Printfail/filament-encoder-calibration.git"
REPO_NAME="filament-encoder-calibration"

# Farben
OFF='\033[0m'
B_RED='\033[1;31m'
B_GREEN='\033[1;32m'
B_YELLOW='\033[1;33m'
B_CYAN='\033[1;36m'
B_WHITE='\033[1;37m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
DIM='\033[2m'

# Pfade
SCRIPTPATH="$(dirname "$(readlink -f "$0")")"
KLIPPER_HOME="${HOME}/klipper"
KLIPPER_CONFIG_HOME="${HOME}/printer_data/config"
KLIPPER_EXTRAS="${KLIPPER_HOME}/klippy/extras"
CONFIG_DIR="${KLIPPER_CONFIG_HOME}/Encoder"

# GitHub Installation: wenn wir nicht im Repo (kein extras/encoder_calibration.py)
if [ ! -f "${SCRIPTPATH}/extras/encoder_calibration.py" ]; then
    INSTALL_DIR="${HOME}/${REPO_NAME}"
    echo -e "${B_CYAN}═══════════════════════════════════════════════════════${OFF}"
    echo -e "${B_CYAN}  GitHub Installation${OFF}"
    echo -e "${B_CYAN}═══════════════════════════════════════════════════════${OFF}"
    echo ""

    if [ -d "${INSTALL_DIR}" ]; then
        echo -e "${B_YELLOW}⚠${OFF} Repository existiert bereits in ${INSTALL_DIR}"
        read -p "$(echo -e ${CYAN}Git Pull für Update? \(y/n\): ${OFF})" yn
        case $yn in
            [JjYy]* )
                cd "${INSTALL_DIR}"
                git pull
                ;;
            * )
                echo -e "${CYAN}ℹ${OFF} Verwende existierendes Repository"
                ;;
        esac
    else
        echo -e "${B_GREEN}➜${OFF} Klone Repository von GitHub..."
        git clone "${GITHUB_REPO}" "${INSTALL_DIR}"
        if [ $? -ne 0 ]; then
            echo -e "${B_RED}✗${OFF} Git Clone fehlgeschlagen!"
            exit 1
        fi
        echo -e "${B_GREEN}✓${OFF} Repository geklont"
    fi

    echo ""
    echo -e "${B_GREEN}➜${OFF} Starte Installation aus Repository..."
    cd "${INSTALL_DIR}"
    chmod +x install.sh 2>/dev/null || true
    bash ./install.sh
    exit 0
fi

SRCDIR="${SCRIPTPATH}"

show_logo() {
    clear
    echo -e "${B_CYAN}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo -e "${B_CYAN}║${B_WHITE}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_GREEN}         ███████╗███╗   ██╗ ██████╗ ██████╗ ██████╗ ███████╗ ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_GREEN}         ██╔════╝████╗  ██║██╔════╝ ██╔══██╗██╔════╝ ██╔════╝ ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_GREEN}         █████╗  ██╔██╗ ██║██║  ███╗██████╔╝██║  ███╗█████╗   ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_YELLOW}         ██╔══╝  ██║╚██╗██║██║   ██║██╔══██╗██║   ██║██╔══╝   ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_YELLOW}         ███████╗██║ ╚████║╚██████╔╝██║  ██║╚██████╔╝███████╗ ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_YELLOW}         ╚══════╝╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝ ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_WHITE}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_CYAN}           Pico W Filament ENCODER + Hall-Sensor            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_WHITE}                    Version ${VERSION}                           ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${B_WHITE}                                                            ${B_CYAN}║${OFF}"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${OFF}\n"
}


#####################################################################
# HELPER FUNKTIONEN
#####################################################################

print_msg()      { echo -e "${B_GREEN}➜${OFF} $1"; }
print_info()     { echo -e "${CYAN}ℹ${OFF} $1"; }
print_warning()  { echo -e "${B_YELLOW}⚠${OFF} $1"; }
print_error()    { echo -e "${B_RED}✗${OFF} $1"; }
print_success()  { echo -e "${B_GREEN}✓${OFF} $1"; }

ask_yn() {
    if [ "${NON_INTERACTIVE}" = "1" ]; then
        return 0
    fi
    while true; do
        read -p "$(echo -e ${CYAN}$1 \(y/n\): ${OFF})" yn
        case $yn in
            [JjYy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Bitte 'y' oder 'n' eingeben.";;
        esac
    done
}

#####################################################################
# PRÜFUNGEN
#####################################################################

check_klipper() {
    if [ ! -d "${KLIPPER_EXTRAS}" ]; then
        print_error "Klipper nicht gefunden: ${KLIPPER_EXTRAS}"
        exit 1
    fi
    print_success "Klipper gefunden: ${KLIPPER_HOME}"
}

check_source_files() {
    if [ ! -f "${SRCDIR}/extras/encoder_calibration.py" ]; then
        print_error "encoder_calibration.py nicht gefunden!"
        exit 1
    fi
    if [ ! -f "${SRCDIR}/extras/filament_width_sensor_corrected.py" ]; then
        print_error "filament_width_sensor_corrected.py nicht gefunden!"
        exit 1
    fi
    if [ ! -f "${SRCDIR}/config/encoder_calibration.cfg" ]; then
        print_error "encoder_calibration.cfg nicht gefunden!"
        exit 1
    fi
    if [ ! -f "${SRCDIR}/config/rotation_distance.cfg" ]; then
        print_error "rotation_distance.cfg nicht gefunden!"
        exit 1
    fi
    print_success "Quell-Dateien gefunden"
}

#####################################################################
# INSTALLATION
#####################################################################

do_install() {
    if [ "${NON_INTERACTIVE}" != "1" ]; then
        show_logo
        echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
        echo -e "${B_YELLOW}  INSTALLATION${OFF}"
        echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
        echo ""
    fi

    print_msg "Prüfe Voraussetzungen..."
    check_klipper
    check_source_files
    echo ""

    # Python-Module (Symlinks)
    print_msg "Installiere Python-Module..."
    
    # encoder_calibration.py
    if [ -L "${KLIPPER_EXTRAS}/encoder_calibration.py" ] || [ -f "${KLIPPER_EXTRAS}/encoder_calibration.py" ]; then
        rm -f "${KLIPPER_EXTRAS}/encoder_calibration.py"
    fi
    ln -sf "${SRCDIR}/extras/encoder_calibration.py" "${KLIPPER_EXTRAS}/encoder_calibration.py"
    print_success "encoder_calibration.py verlinkt"
    
    # filament_width_sensor_corrected.py
    if [ -L "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py" ] || [ -f "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py" ]; then
        rm -f "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py"
    fi
    ln -sf "${SRCDIR}/extras/filament_width_sensor_corrected.py" "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py"
    print_success "filament_width_sensor_corrected.py verlinkt"
    echo ""
    
    # WICHTIG: Lösche Python-Caches um alte Versionen zu entfernen
    print_msg "Lösche Python-Caches..."
    rm -rf "${KLIPPER_EXTRAS}/__pycache__/encoder_calibration*.pyc" 2>/dev/null
    rm -rf "${KLIPPER_EXTRAS}/__pycache__/filament_width_sensor_corrected*.pyc" 2>/dev/null
    rm -rf "${KLIPPER_HOME}/klippy/__pycache__" 2>/dev/null
    print_success "Caches gelöscht"
    echo ""

    # Config-Ordner
    if [ ! -d "${CONFIG_DIR}" ]; then
        print_msg "Erstelle Config-Ordner..."
        mkdir -p "${CONFIG_DIR}"
        chmod 775 "${CONFIG_DIR}"
        print_success "Ordner erstellt: ${CONFIG_DIR}"
    fi

    # encoder_calibration.cfg immer aktualisieren (Makros)
    print_msg "Installiere/aktualisiere encoder_calibration.cfg..."
    cp -f "${SRCDIR}/config/encoder_calibration.cfg" "${CONFIG_DIR}/"
    chmod 644 "${CONFIG_DIR}/encoder_calibration.cfg"
    print_success "encoder_calibration.cfg installiert/aktualisiert"

    # rotation_distance.cfg nur installieren, wenn nicht vorhanden
    if [ ! -f "${CONFIG_DIR}/rotation_distance.cfg" ]; then
        print_msg "Installiere rotation_distance.cfg..."
        cp "${SRCDIR}/config/rotation_distance.cfg" "${CONFIG_DIR}/"
        chmod 644 "${CONFIG_DIR}/rotation_distance.cfg"
        print_success "rotation_distance.cfg installiert"
    else
        print_warning "rotation_distance.cfg existiert bereits - wird NICHT überschrieben"
    fi
    echo ""

    # Bleak installieren
    print_msg "Installiere/prüfe Python-Abhängigkeit 'bleak'..."
    if [ -x "${HOME}/klippy-env/bin/pip" ]; then
        "${HOME}/klippy-env/bin/pip" install --upgrade bleak
        print_success "bleak installiert/aktualisiert"
    else
        print_warning "klippy-env pip nicht gefunden. Bitte manuell installieren: ~/klippy-env/bin/pip install bleak"
    fi
    echo ""

    print_success "Installation abgeschlossen!"
    echo ""
    print_info "Füge in deine printer.cfg hinzu:"
    print_info "  [include Encoder/*.cfg]"
    print_info ""
    print_info "Alternative (einzeln):"
    print_info "  [include Encoder/encoder_calibration.cfg]"
    print_info "  [include Encoder/rotation_distance.cfg]"
    echo ""

    if ask_yn "Klipper jetzt neu starten?"; then
        print_msg "Starte Klipper neu..."
        sudo systemctl restart klipper
        sleep 2
        print_success "Klipper neu gestartet!"
    fi
    echo ""
}

#####################################################################
# UPDATE
#####################################################################

do_update() {
    show_logo
    echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
    echo -e "${B_YELLOW}  UPDATE${OFF}"
    echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
    echo ""

    print_msg "Aktualisiere Python-Module (Symlinks)..."
    
    # encoder_calibration.py
    rm -f "${KLIPPER_EXTRAS}/encoder_calibration.py"
    ln -sf "${SRCDIR}/extras/encoder_calibration.py" "${KLIPPER_EXTRAS}/encoder_calibration.py"
    print_success "encoder_calibration.py aktualisiert"
    
    # filament_width_sensor_corrected.py
    rm -f "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py"
    ln -sf "${SRCDIR}/extras/filament_width_sensor_corrected.py" "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py"
    print_success "filament_width_sensor_corrected.py aktualisiert"
    echo ""
    
    # WICHTIG: Lösche Python-Caches um alte Versionen zu entfernen
    print_msg "Lösche Python-Caches..."
    rm -rf "${KLIPPER_EXTRAS}/__pycache__/encoder_calibration*.pyc" 2>/dev/null
    rm -rf "${KLIPPER_EXTRAS}/__pycache__/filament_width_sensor_corrected*.pyc" 2>/dev/null
    rm -rf "${KLIPPER_HOME}/klippy/__pycache__" 2>/dev/null
    print_success "Caches gelöscht"
    echo ""

    print_msg "Aktualisiere encoder_calibration.cfg (Makros)..."
    cp -f "${SRCDIR}/config/encoder_calibration.cfg" "${CONFIG_DIR}/"
    chmod 644 "${CONFIG_DIR}/encoder_calibration.cfg"
    print_success "encoder_calibration.cfg aktualisiert"
    echo ""

    print_warning "rotation_distance.cfg wird NICHT automatisch überschrieben (falls du sie angepasst hast)."
    echo ""

    if ask_yn "Klipper jetzt neu starten?"; then
        print_msg "Starte Klipper neu..."
        sudo systemctl restart klipper
        sleep 2
        print_success "Klipper neu gestartet!"
    fi
    echo ""
}

#####################################################################
# DEINSTALLATION
#####################################################################

do_uninstall() {
    show_logo
    echo -e "${B_RED}═══════════════════════════════════════════════════════${OFF}"
    echo -e "${B_RED}  DEINSTALLATION${OFF}"
    echo -e "${B_RED}═══════════════════════════════════════════════════════${OFF}"
    echo ""

    print_warning "Dies wird das Encoder-Modul vollständig entfernen!"
    echo ""

    if ! ask_yn "Wirklich deinstallieren?"; then
        print_info "Abgebrochen"
        return
    fi
    echo ""

    # Python-Module
    print_msg "Entferne Python-Module..."
    if [ -L "${KLIPPER_EXTRAS}/encoder_calibration.py" ] || [ -f "${KLIPPER_EXTRAS}/encoder_calibration.py" ]; then
        rm -f "${KLIPPER_EXTRAS}/encoder_calibration.py"
        print_success "encoder_calibration.py entfernt"
    fi
    if [ -L "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py" ] || [ -f "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py" ]; then
        rm -f "${KLIPPER_EXTRAS}/filament_width_sensor_corrected.py"
        print_success "filament_width_sensor_corrected.py entfernt"
    fi

    # Config-Ordner optional löschen
    if [ -d "${CONFIG_DIR}" ]; then
        echo ""
        if ask_yn "Auch Config-Ordner ${CONFIG_DIR} löschen? (Einstellungen gehen verloren!)"; then
            print_msg "Entferne Config-Ordner..."
            rm -rf "${CONFIG_DIR}"
            print_success "Config entfernt"
        else
            print_info "Config bleibt erhalten"
        fi
    fi

    echo ""
    print_success "Deinstallation abgeschlossen!"
    echo ""
    print_warning "Entferne folgende Zeile aus printer.cfg (falls vorhanden):"
    print_warning "  [include Encoder/encoder_calibration.cfg]"
    echo ""

    if ask_yn "Klipper jetzt neu starten?"; then
        print_msg "Starte Klipper neu..."
        sudo systemctl restart klipper
        sleep 2
        print_success "Klipper neu gestartet!"
    fi
    echo ""
}

#####################################################################
# STATUS
#####################################################################

show_status() {
    show_logo
    echo -e "${B_WHITE}═══════════════════════════════════════════════════════${OFF}"
    echo -e "${B_WHITE}  STATUS${OFF}"
    echo -e "${B_WHITE}═══════════════════════════════════════════════════════${OFF}"
    echo ""

    # Python-Modul
    if [ -L "${KLIPPER_EXTRAS}/encoder_calibration.py" ]; then
        print_success "Python-Modul: Installiert (Symlink)"
        print_info "  Link: $(readlink "${KLIPPER_EXTRAS}/encoder_calibration.py")"
    elif [ -f "${KLIPPER_EXTRAS}/encoder_calibration.py" ]; then
        print_warning "Python-Modul: Installiert (Datei, kein Symlink!)"
    else
        print_error "Python-Modul: Nicht installiert"
    fi

    # Configs
    if [ -f "${CONFIG_DIR}/encoder_calibration.cfg" ]; then
        print_success "encoder_calibration.cfg: Installiert"
        print_info "  Pfad: ${CONFIG_DIR}/encoder_calibration.cfg"
    else
        print_error "encoder_calibration.cfg: Nicht installiert"
    fi

    if [ -f "${CONFIG_DIR}/rotation_distance.cfg" ]; then
        print_success "rotation_distance.cfg: Installiert"
        print_info "  Pfad: ${CONFIG_DIR}/rotation_distance.cfg"
    else
        print_warning "rotation_distance.cfg: Nicht installiert"
    fi

    # Klipper
    if systemctl is-active --quiet klipper; then
        print_success "Klipper: Läuft"
    else
        print_error "Klipper: Gestoppt"
    fi

    echo ""
}

#####################################################################
# MENÜ
#####################################################################

show_menu() {
    show_logo
    echo -e "${B_CYAN}╔════════════════════════════════════════════════════════════╗${OFF}"
    echo -e "${B_CYAN}║${B_WHITE}                        HAUPT-MENÜ                          ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}╠════════════════════════════════════════════════════════════╣${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}  ${B_GREEN}1)${OFF} Neu installieren                                       ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}     ${DIM}Erstinstallation / Symlink-Setup${OFF}                      ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}  ${B_YELLOW}2)${OFF} Update / Re-installieren                               ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}     ${DIM}Aktualisiert Code, behält Config${OFF}                       ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}  ${B_RED}3)${OFF} Deinstallieren                                         ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}     ${DIM}Entfernt Modul und optional Config${OFF}                      ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}  ${B_CYAN}4)${OFF} Status anzeigen                                        ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}     ${DIM}Prüft Installation${OFF}                                    ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}  ${B_WHITE}5)${OFF} Beenden                                                ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}║${OFF}                                                            ${B_CYAN}║${OFF}"
    echo -e "${B_CYAN}╚════════════════════════════════════════════════════════════╝${OFF}"
    echo ""
}

#####################################################################
# HAUPTPROGRAMM
#####################################################################

main() {
    while true; do
        show_menu
        read -p "$(echo -e ${CYAN}Wähle eine Option \[1-5\]: ${OFF})" choice
        echo ""

        case $choice in
            1)
                do_install
                read -p "Drücke Enter zum Fortfahren..." _
                ;;
            2)
                do_update
                read -p "Drücke Enter zum Fortfahren..." _
                ;;
            3)
                do_uninstall
                read -p "Drücke Enter zum Fortfahren..." _
                ;;
            4)
                show_status
                read -p "Drücke Enter zum Fortfahren..." _
                ;;
            5)
                show_logo
                print_success "Auf Wiedersehen!"
                echo ""
                exit 0
                ;;
            *)
                print_error "Ungültige Auswahl!"
                sleep 1
                ;;
        esac
    done
}

# Script starten
if [ -t 0 ]; then
    main
else
    export NON_INTERACTIVE=1
    show_logo
    echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
    echo -e "${B_YELLOW}  AUTO-INSTALLATION (One-Liner)${OFF}"
    echo -e "${B_YELLOW}═══════════════════════════════════════════════════════${OFF}"
    echo ""
    print_info "Keine interaktive Shell erkannt - starte automatische Installation..."
    echo ""
    sleep 1
    do_install
fi

