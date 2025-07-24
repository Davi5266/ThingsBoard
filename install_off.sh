cd ./install_offline/
# carregando imagens
docker load -i postgres_16.tar
docker load -i thingsboard.tar
docker load -i adminer.tar
docker load -i zookeeper.tar
docker load -i kafka.tar
cd ../
./install.sh
