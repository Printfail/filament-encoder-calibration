#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// lwIP deaktiviert - nur um Include-Fehler zu vermeiden
// Wir nutzen kein WiFi/Netzwerk, nur BLE

#define NO_SYS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0
#define LWIP_NETIF_API 0
#define LWIP_NETIF_STATUS_CALLBACK 0

#endif // LWIPOPTS.H
