function get_ip() {
    local hostnames=($(hostname -I))
    if [ -z "$hostnames" ]; then
        echo "127.0.0.1"
    else
        for i in "${hostnames[@]}"; do
            if [[ $i =~ ^([0-9]{1,2}|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\.([0-9]{1,2}|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\.([0-9]{1,2}|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\.([0-9]{1,2}|1[0-9][0-9]|2[0-4][0-9]|25[0-5])$ ]]; then
                addr_ipv4=$i
            elif [[ $i =~ ^(([0-9a-fA-F]{1,4}:){7,7}[0-9a-fA-F]{1,4}|([0-9a-fA-F]{1,4}:){1,7}:|([0-9a-fA-F]{1,4}:){1,6}:[0-9a-fA-F]{1,4}|([0-9a-fA-F]{1,4}:){1,5}(:[0-9a-fA-F]{1,4}){1,2}|([0-9a-fA-F]{1,4}:){1,4}(:[0-9a-fA-F]{1,4}){1,3}|([0-9a-fA-F]{1,4}:){1,3}(:[0-9a-fA-F]{1,4}){1,4}|([0-9a-fA-F]{1,4}:){1,2}(:[0-9a-fA-F]{1,4}){1,5}|[0-9a-fA-F]{1,4}:((:[0-9a-fA-F]{1,4}){1,6})|:((:[0-9a-fA-F]{1,4}){1,7}|:)|fe80:(:[0-9a-fA-F]{0,4}){0,4}%[0-9a-zA-Z]{1,}|::(ffff(:0{1,4}){0,1}:){0,1}((25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])\.){3,3}(25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])|([0-9a-fA-F]{1,4}:){1,4}:((25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])\.){3,3}(25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9]))$ ]]; then
                addr_ipv6=$i
            fi
        done
        if [ -z $1 ]; then
            echo ${hostnames[0]}
        elif [ $1 = "ipv4" ]; then
            if [ -n "$addr_ipv4" ]; then
                echo "$addr_ipv4"
            else
                echo "127.0.0.1"
            fi
        elif [ $1 = "ipv6" ]; then
            if [ -n "$addr_ipv6" ]; then
                echo "$addr_ipv6"
            else
                echo "::"
            fi
        fi
    fi
}
alias get_ip=get_ip

function change_master(){
    OLD_MASTER_IP=$MASTER_IP
    sed -i '/MASTER_IP=/c\export MASTER_IP=$1' ~/.rosrc
    . ~/.rosrc
    echo "Master changed from $OLD_MASTER_IP to $MASTER_IP"
}
alias change_master=change_master
