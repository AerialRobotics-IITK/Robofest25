# Swarm System ARIITK

## Note:
1. Use Zenoh Router instead of default DDS,default DDS chokes the system and is not made for mesh network
2. Remove the ip address of the system on which zenoh router is running from `router_config.json5`
3. Give a unique `tgt_system` parameter in mavros launch command and `SYSID_THISMAV` parameter for each drone


