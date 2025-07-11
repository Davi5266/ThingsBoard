#!/bin/bash
while true
do
	curl -v -X POST http://192.168.0.113:8080/api/v1/9eqdGc5Jv8zweLVEu4lP/telemetry --header Content-Type:application/json --data "{temperature:66}"
	sleep 1

	curl -v -X POST http://192.168.0.113:8080/api/v1/9eqdGc5Jv8zweLVEu4lP/telemetry --header Content-Type:application/json --data "{temperature:56}"

	sleep 1

	curl -v -X POST http://192.168.0.113:8080/api/v1/9eqdGc5Jv8zweLVEu4lP/telemetry --header Content-Type:application/json --data "{temperature:89}"

	sleep 1

done
