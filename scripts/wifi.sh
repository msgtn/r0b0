#!/bin/bash
# Sourced from https://github.com/pi-top/Wi-Fi-Access-Point-and-Station-Mode
###############################################################
#                Unofficial 'Bash strict mode'                #
# http://redsymbol.net/articles/unofficial-bash-strict-mode/  #
###############################################################
set -euo pipefail
IFS=$'\n\t'
###############################################################

AP_MODE_CONFIG_FILE="/etc/default/wifi-ap-sta"

wifi_country() {
	local country
	country=$(wpa_cli -i "$WIFI_IFACE" get country 2>/dev/null)
	if [ "${country}" = "FAIL" ] && [ -x "$(command -v raspi-config)" ]; then
		country=$(raspi-config nonint get_wifi_country)
	else
		country="US"
	fi
	echo "${country:-US}"
}

initialise_ap_ssid_and_password() {
	AP_SSID="${AP_SSID:-$(produce_ssid)}"
	AP_PASSWORD="${AP_PASSWORD:-$(produce_password)}"
}

initialise_ap_values() {
	ETH_IFACE="${ETH_IFACE:-eth0}"
	AP_IFACE="${AP_IFACE:-wlan_ap0}"
	AP_MAC="${AP_MAC:-99:88:77:66:55:44}"
	WIFI_IFACE="${WIFI_IFACE:-wlan0}"
	STATIC_IP_PREFIX="${STATIC_IP_PREFIX:-192.168.90}"
	SUBNET="${SUBNET:-24}"
	IFACE_IP="${IFACE_IP:-${STATIC_IP_PREFIX}.1}"
	SUBNET_IP="${SUBNET_IP:-${STATIC_IP_PREFIX}.0}"
	SUBNET_MASK="${SUBNET_MASK:-255.255.255.0}"
	DHCP_RANGE_START="${DHCP_RANGE_START:-10}"
	DHCP_RANGE_END="${DHCP_RANGE_END:-50}"
	CONNECTION_NAME="${CONNECTION_NAME:-pi-top-ap}"
}

produce_ssid() {
	local SSID
	if [ -f "/proc/cpuinfo" ]; then
		SSID=$(grep "Serial" /proc/cpuinfo | awk '{print $3}')
	fi
	SSID="${SSID:-1234}"
	SSID=$(echo "${SSID: -4}" | tr '[:lower:]' '[:upper:]')
	echo "$(hostname)-${SSID}"
}

produce_password() {
	local PASSPHRASE;
	PASSPHRASE="$(< /dev/urandom tr -dc a-km-z0-9 | head -c8)"
	echo "AP_PASSWORD=${PASSPHRASE}" >> "${AP_MODE_CONFIG_FILE}"
	echo "${PASSPHRASE}"
}

usage_and_exit() {
	echo "Application that sets up a Raspberry Pi as a wireless access point."
	echo "Customize the access point settings by editing ${AP_MODE_CONFIG_FILE}"
	echo ""
	echo -e "Usage:\\n\\t${0} {start,stop,status}"
	echo "where:"
	echo -e "\\tstart\\t: use Raspberry Pi as wireless access point."
	echo -e "\\tstop\\t: stop access point mode and restore previous configuration."
	echo -e "\\tstatus\\t: display access point state and information."
	echo -e "\\twait_interface\\t: wait until access point interface is ready."
	echo ""
	exit 1
}

if [[ "$EUID" -ne 0 ]]; then
	echo "Please run as root"
	exit 1
fi

command=${1:-}
valid_commands=("configure" "start" "stop" "status" "enable" "disable" "wait_interface")
if [[ -z "${command}" ]] || [[ ! "${valid_commands[*]}" =~ ${command} ]]; then
	# shellcheck disable=SC2145
	echo "Invalid argument: accepted values are: ${valid_commands[@]}."
	echo ""
	usage_and_exit
fi

# shellcheck disable=SC1090
[[ -f "${AP_MODE_CONFIG_FILE}" ]] && source "${AP_MODE_CONFIG_FILE}"

initialise_ap_values

main() {
	if [[ "${command}" == "configure" ]]; then
		do_configure
	elif [[ "${command}" == "start" ]]; then
		do_start
	elif [[ "${command}" == "stop" ]]; then
		do_stop
	elif [[ "${command}" == "enable" ]]; then
		do_enable
	elif [[ "${command}" == "disable" ]]; then
		do_disable
	elif [[ "${command}" == "wait_interface" ]]; then
		do_wait_interface
		exit
	else
		:
	fi
	print_current_status
}

do_configure() {
	_enable_wireless_card_if_necessary
	_stop_ap_mode_if_active
	_create_virtual_interface
	_wait_for_virtual_interface
	initialise_ap_ssid_and_password
	_enable_wireless_network_passthrough
	_create_network_manager_connection
	_start_ap_mode
	_wait_for_virtual_interface_ip_addr
}

do_enable() {
	_enable_wireless_card_if_necessary
	_create_virtual_interface_udev_rule
	_add_pre_conditions_on_network_manager_service_start
}

do_disable() {
	_disable_wireless_network_passthrough
	_remove_virtual_interface_udev_rule
	_remove_configure_on_network_manager_service_start
}

do_start() {
	do_configure
	_add_pre_conditions_on_network_manager_service_start
	_reload_systemd_daemon
}

do_stop() {
	_stop_ap_mode_if_active
	_disable_wireless_network_passthrough
	_remove_virtual_interface
	_remove_configure_on_network_manager_service_start
	_reload_systemd_daemon
	_start_wifi_managed_mode
}

do_wait_interface() {
	_wait_for_virtual_interface
	_wait_for_virtual_interface_ip_addr
}

print_current_status() {
	if ! nmcli con show --active | grep -q "${CONNECTION_NAME}" &>/dev/null; then
		state="inactive"
		echo "State: ${state}"
		return
	fi

	state="active"
	config="$(nmcli con show --active | grep "${CONNECTION_NAME}")"
	iface="$(echo "${config}" | head -n1 | awk '{print $NF}' | tr -d "'")"
	ip_addr="$(ip -f inet addr show "${iface}" | sed -En -e 's/.*inet ([0-9.]+).*/\1/p')"

	if [[ -z "${ip_addr:-}" ]]; then
		echo "State: ${state}"
		return
	fi

	echo "State: ${state}"

	ssid=$(nmcli -t -f 802-11-wireless.ssid con show "${CONNECTION_NAME}" | head -n1 | cut -d':' -f2)
	echo -e "Access Point Network SSID:\\t${ssid}"
	echo -e "Access Point Wi-Fi Password:\\t${AP_PASSWORD}"
	echo -e "Access Point IP Address:\\t${ip_addr}"
}

###########################################
# NETWORK-MANAGER SERVICE FILE MANAGEMENT #
###########################################
_add_pre_conditions_on_network_manager_service_start() {
	rm -f /lib/systemd/system/NetworkManager.service.d/10-wifi-ap-sta.conf
	mkdir -p /lib/systemd/system/NetworkManager.service.d
	cat <<EOF >/lib/systemd/system/NetworkManager.service.d/10-wifi-ap-sta.conf
[Service]
ExecStartPost=/usr/bin/wifi-ap-sta configure
Restart=on-failure
EOF
}

_remove_configure_on_network_manager_service_start() {
	SERVICE_DROP_IN_FILE="/lib/systemd/system/NetworkManager.service.d/10-wifi-ap-sta.conf"
	if [[ -f "${SERVICE_DROP_IN_FILE}" ]]; then
		echo "Removing drop-in unit '${SERVICE_DROP_IN_FILE}'..."
		rm "${SERVICE_DROP_IN_FILE}"
	fi
}

_reload_systemd_daemon() {
	systemctl daemon-reload
}

_enable_wireless_network_passthrough() {
	echo 1 >/proc/sys/net/ipv4/ip_forward
	iptables -t nat -A POSTROUTING -o "${WIFI_IFACE}" -j MASQUERADE
	iptables -t nat -A POSTROUTING -o "${ETH_IFACE}" -j MASQUERADE
}

_disable_wireless_network_passthrough() {
	echo 0 >/proc/sys/net/ipv4/ip_forward
	# We want these commands to succeed even if the rule doesn't exist.
	# This could happen when running the command after upgrading the package.
	iptables -t nat -D POSTROUTING -o "${WIFI_IFACE}" -j MASQUERADE || true
	iptables -t nat -D POSTROUTING -o "${ETH_IFACE}" -j MASQUERADE || true
}


#################################
# PHYSICAL INTERFACE MANAGEMENT #
#################################
_enable_wireless_card_if_necessary() {
	wifi_state=$(rfkill list wifi -o Soft -n)
	if [[ "${wifi_state}" == "blocked" ]]; then
		echo "Wireless card is disabled - enabling ..."
		rfkill unblock wifi
	fi
}

################################################################
UDEV_RULES_FILE="/etc/udev/rules.d/70-persistent-net.rules"

_reload_udev_rules() {
	udevadm control --reload-rules
	udevadm trigger
}

_create_virtual_interface_udev_rule() {
	cat >"${UDEV_RULES_FILE}" <<EOF
SUBSYSTEM=="ieee80211", ACTION=="add|change", KERNEL=="phy0", \
RUN+="/sbin/iw phy phy0 interface add ${AP_IFACE} type __ap", \
RUN+="/bin/ip link set ${AP_IFACE} address ${AP_MAC}"
EOF
}

_create_virtual_interface() {
	echo "Creating virtual interface '${AP_IFACE}'..."
	_create_virtual_interface_udev_rule
	_reload_udev_rules
}

_wait_for_virtual_interface() {
	if ! ip a l "${AP_IFACE}" &>/dev/null; then
		echo "Interface '${AP_IFACE}' isn't ready - waiting..."
		while ! ip a l "${AP_IFACE}" &>/dev/null; do
			sleep 1
		done
	fi
}

_wait_for_virtual_interface_ip_addr() {
	if [[ -z "$(ip -f inet addr show "${AP_IFACE}" | sed -En -e 's/.*inet ([0-9.]+).*/\1/p')" ]]; then
		SLEEP_COUNTER=0
		echo "Interface '${AP_IFACE}' does not yet have an IP address - waiting..."
		while [[ -z "$(ip -f inet addr show "${AP_IFACE}" | sed -En -e 's/.*inet ([0-9.]+).*/\1/p')" ]]; do
			sleep 1
			if [ $SLEEP_COUNTER -eq 5 ]; then
				echo "Trying to reconfigure interface ${AP_IFACE} after waiting for ${SLEEP_COUNTER} seconds"
				_set_ap_interface_static_ip
			elif [ $SLEEP_COUNTER -eq 10 ]; then
				echo "Timeout waiting for ${AP_IFACE} to get an IP address after ${SLEEP_COUNTER}, exiting..."
				exit 1
			fi
			SLEEP_COUNTER=$[$SLEEP_COUNTER +1]
		done
	fi
}

_remove_virtual_interface_udev_rule() {
	if [[ -f "${UDEV_RULES_FILE}" ]]; then
		echo "Removing virtual interface '${AP_IFACE}'..."
		rm "${UDEV_RULES_FILE}"
	fi
}

_remove_virtual_interface() {
	_remove_virtual_interface_udev_rule
	_reload_udev_rules
	if iw dev | grep -q "${AP_IFACE}"; then
		iw dev "${AP_IFACE}" del
	fi
}

########################
# WIFI MODE MANAGEMENT #
########################
_set_ap_interface_static_ip() {
	ifconfig "${AP_IFACE}" "${IFACE_IP}" netmask "${SUBNET_MASK}"
	ifconfig "${AP_IFACE}" up
}

_start_ap_mode() {
	echo "Starting Wifi: Access Point Mode ..."

	_set_ap_interface_static_ip
	_start_network_manager_connection

	# wpa_cli -i "${WIFI_IFACE}" reconfigure &>/dev/null
	echo "Started AP mode with SSID ${AP_SSID}"
}

_stop_ap_mode_if_active() {
	echo "Stopping Wifi: Access Point Mode ..."
	nmcli con down "${CONNECTION_NAME}" &>/dev/null || true
	nmcli con delete "${CONNECTION_NAME}" &>/dev/null || true
	echo "Stopped AP mode"
	# wpa_cli -i "${WIFI_IFACE}" reconfigure &>/dev/null
}

_start_wifi_managed_mode() {
	echo "Starting Wifi: Managed Mode ..."
	dhclient -r "${WIFI_IFACE}"
}

_package_is_installed() {
	[[ $(dpkg-query --show --showformat='${db:Status-Status}\n' "${1}" 2>/dev/null) = installed ]]
}

_create_network_manager_connection() {
	nmcli con add con-name "${CONNECTION_NAME}" \
		ifname "${AP_IFACE}" \
		type wifi \
		ssid "${AP_SSID}" \
		wifi-sec.key-mgmt wpa-psk \
		wifi-sec.psk "${AP_PASSWORD}" \
		802-11-wireless.mode ap \
		802-11-wireless.band bg \
		ipv4.method shared \
		ipv4.addresses "${IFACE_IP}/${SUBNET}"
}

_start_network_manager_connection() {
	nmcli con up "${CONNECTION_NAME}"
}

main
