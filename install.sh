# Initialize database schema & system assets
#docker compose run --rm -e INSTALL_TB=true -e LOAD_DEMO=true thingsboard-ce

mkdir -p ./.mytb-data && sudo chown -R 799:799 ./.mytb-data
mkdir -p ./.mytb-logs && sudo chown -R 799:799 ./.mytb-logs
docker run -it -p 8080:9090 -p 7070:7070 -p 1883:1883 -p 5683-5688:5683-5688/udp -p 5432:5432 -v ./.mytb-data:/data \
-v ./.mytb-logs:/var/log/thingsboard --name mytb --restart always thingsboard/tb-postgres
#docker run -it -v ~/.mytb-data:/data \
#-v ~/.mytb-logs:/var/log/thingsboard --name mytb --restart always thingsboard/tb-postgres

# Start the platform & tail logs

#docker compose up -d && docker compose logs -f thingsboard-ce
