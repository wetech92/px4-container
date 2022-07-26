1. Check if there is an intended `msg` in original repository's `/msg`
2. If it is not present, add it and add it to `/msg/CMakeLists.txt`
3. Add decision about whether you will receive or send that message in `/msg/tools/urtps_bridge_topics.yaml`
4. Rebuild container from outside or manually apply new msgs and rebuild px4_ros in container. You are go.