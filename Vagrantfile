# -*- mode: ruby -*-
# vi: set ft=ruby :

ENV["LC_ALL"] = "en_US.UTF-8"

## Set which virtual box
UBUNTU_BOX = "bento/ubuntu-22.04" # https://portal.cloud.hashicorp.com/vagrant/discover/bento/ubuntu-22.04
# UBUNTU_BOX = "bento/ubuntu-24.04"

Vagrant.configure("2") do |config|
  # The most common configuration options are documented and commented below.
  # For a complete reference, please see the online documentation at
  # https://docs.vagrantup.com.

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine. In the example below,
  # accessing "localhost:8080" will access port 80 on the guest machine.
  # NOTE: This will enable public access to the opened port
  # config.vm.network "forwarded_port", guest: 80, host: 8080

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine and only allow access
  # via 127.0.0.1 to disable public access
  # config.vm.network "forwarded_port", guest: 80, host: 8080, host_ip: "127.0.0.1"

  # ====Shared folder======================
  config.vm.synced_folder ".", "/opt/dockerized-norlab-project",
    type: "rsync",
    rsync__exclude: [".vagrant", ".idea", ".DS_Store"],
    rsync__verbose: false,
    rsync__auto: true,
    automount: true,
    disabled: false

  # ====vagrant-gatling-rsync==============
  ##   Ref https://github.com/smerrill/vagrant-gatling-rsync
  #if Vagrant.has_plugin?("vagrant-gatling-rsync")
  #  config.gatling.latency = 2.5
  #  config.gatling.time_format = "%H:%M:%S"
  #end

  # Automatically start the sync engine on vagrant up or vagrant reload
  # when the machines that you bring up have one or more rsync folders defined
  config.gatling.rsync_on_startup = false

  # ====Provider-specific configuration====
  #  Parallels Tools customization with prlctl command-line utility:
  #  Ref:
  #  - https://parallels.github.io/vagrant-parallels/docs/configuration.html
  #  - https://download.parallels.com/desktop/v16/docs/en_US/Parallels%20Desktop%20Pro%20Edition%20Command-Line%20Reference.pdf
  #
  config.vm.provider "parallels" do |prl|
    prl.name = "dockerized-norlab-project-DEV"

    # Customize the amount of memory on the VM:
    prl.memory = 16048
    prl.cpus = 4

    # Setup Parallels Tools Auto-Update
    prl.update_guest_tools = true

    # VM portability
    prl.linked_clone = true # (CRITICAL) ToDo: Switch to false for release (!)

    #prl.customize "post-import", ["set", :id,
    #   "--description", "Dockerized-Norlab-Project UX validation VM"
    #   ]
    ##    "--startup-view", "headless",
    ## GUI cutomizations: --startup-view [window, fullscreen, headless, same] --3d-accelerate highest --high-resolution on
    ## Optimization cutomizations: --faster-vm on --resource-quota unlimited
  end

  # Note:
  # - VM port mapping:
  #     - port 80 (mapped to host 8080) is for docker internet access, e.g. executin apt-get update inside docker container
  #     - port 2222 is for accessing the VM ssh server for Remote Developement setup
  # - Private network
  #     - Parallel desktop address range preference:
  #         - shared:  10.211.55.1 <-> 10.211.55.254
  #         - host-only: 10.37.129.1 <-> 10.37.129.254
  #     - Note:
  #         - Private network address range 10.0.0.0 – 10.255.255.255
  #         - ref:
  #             - https://developer.hashicorp.com/vagrant/docs/networking/private_network#static-ip
  #             - https://en.wikipedia.org/wiki/Private_network#Private_IPv4_address_spaces
  config.vm.define "dockerized-norlab-project-vm", primary: true do |dnp|
    dnp.vm.box = UBUNTU_BOX

    # Add these SSH configurations
    dnp.ssh.insert_key = true
    dnp.ssh.username = "vagrant"
    dnp.ssh.password = "vagrant"

    #dnp.vm.hostname = "redleader"
    #dnp.vm.network "private_network", ip: "10.211.55.99"
    #config.vm.network "forwarded_port", guest: 80, host: 8080, auto_correct: true
    #config.vm.network "forwarded_port", guest: 22, host: 2222
    dnp.vm.provision "Check network", type: "shell", run: "always", inline: <<-SHELL
       echo
       echo "I am '$(whoami)', my address is $(ifconfig eth0 | grep inet | awk '$1==\"inet\" {print $2}')"
       echo
    SHELL
  end

  # ====Provisioning========================
  $INLINE_SCRIPT = <<-'SCRIPT'
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install --assume-yes \
     locales \
     sudo \
     apt-utils \
     lsb-release \
     ca-certificates \
     software-properties-common \
     build-essential \
     bash-completion \
     fontconfig \
     vim \
     tree \
     git \
     curl \
     wget \
     gnupg2 \
     zip gzip tar unzip \
     rsync \
     net-tools \
     dnsutils

  # Mock git config
  git config --global user.name "vagrant"
  git config --global user.email "vagrant@gmail.com"
  git config --global init.defaultBranch "main"

#   echo -e "\nCLone dockerized-norlab-project-mock in 'utilities/tmp/'\n"
#   cd /opt/dockerized-norlab-project || exit 1
#   source load_repo_main_dotenv.bash || exit 1
# #   bash tests/setup_mock.bash || exit 1
#   mkdir -p "/opt/dockerized-norlab-project/utilities/tmp/dockerized-norlab-project-mock"
#   git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git "/opt/dockerized-norlab-project/utilities/tmp/dockerized-norlab-project-mock"
#   chown -R $(id -u vagrant) /opt/dockerized-norlab-project/utilities/tmp/dockerized-norlab-project-mock

  echo -e "\nCLone dockerized-norlab-project-mock-EMPTY in '/opt/'\n"
  cd /opt || exit 1
  rm -rf /opt/dockerized-norlab-project-mock-EMPTY
  git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock-EMPTY.git
  chown -R $(id -u vagrant) /opt/dockerized-norlab-project-mock-EMPTY

  # Ressetting
  rm -f /usr/local/bin/dnp
  sed -i '/# >>>> Dockerized-NorLab-Project (start)/,/# <<<< Dockerized-NorLab-Project (end)/d' "${HOME}/.bashrc"
  sed -i '/# >>>> Dockerized-NorLab-Project CUDA (start)/,/# <<<< Dockerized-NorLab-Project CUDA (end)/d' "${HOME}/.bashrc"
  sed -i '/# >>>> Dockerized-NorLab-Project DEV alias (start)/,/# <<<< Dockerized-NorLab-Project DEV alias (end)/d' "${HOME}/.bashrc"

  echo -e "\nSet DEV aliases\n"
  {
      echo "" ;
      echo "# >>>> Dockerized-NorLab-Project DEV alias (start)" ;
      echo "alias dnp-dnp-cd='cd /opt/dockerized-norlab-project'" ;
#       echo "alias dnp-mock-cd='cd /opt/dockerized-norlab-project/utilities/tmp/dockerized-norlab-project-mock'" ;
      echo "alias dnp-mock-empty-cd='cd /opt/dockerized-norlab-project-mock-EMPTY'" ;
      echo "alias dnp-test-symlink='tree -L 1 -a /usr/local/bin/'" ;
      echo "alias dnp-test-bashrc='tail -n 20 ~/.bashrc'" ;
      echo "# <<<< Dockerized-NorLab-Project DEV alias (end)" ;
      echo "" ;
  } >> /home/vagrant/.bashrc

  echo -e "\nSet DEV default landing path to DNP mock super project\n"
  echo "cd /opt/dockerized-norlab-project" >> /home/vagrant/.bashrc

  echo -e "\nInstall The Ubuntu Desktop Gui\n"
  apt-get install --assume-yes --no-install-recommends ubuntu-desktop
  SCRIPT

  config.vm.provision :shell do |shell|
   shell.inline = $INLINE_SCRIPT
   shell.privileged = true
   shell.reboot = true
  end

  # Execute rsync from the host on 'vagrant up' trigger
  config.trigger.after :up do |trigger|
     trigger.only_on = "dockerized-norlab-project-vm"
     trigger.name = "rsync cmd"
     trigger.info = "Executing 'vagrant rsync' now"
     trigger.run = {inline: "bash -c 'vagrant rsync'"}
     trigger.on_error = :continue
  end

  # Execute rsync from the host on 'vagrant snapshot' trigger
  config.trigger.after :snapshot_restore, type: :action do |trigger|
     trigger.only_on = "dockerized-norlab-project-vm"
     trigger.name = "rsync cmd on 'vagrant snapshot restore'"
     trigger.info = "Executing 'vagrant rsync' now"
     trigger.run = {inline: "bash -c 'vagrant rsync'"}
     trigger.on_error = :continue
  end

  config.trigger.after :up do |trigger|
    trigger.only_on = "dockerized-norlab-project-vm"
    trigger.name = "Dir sync info"
    trigger.info = "\033[1;33m Remember to use the command \033[1;2mvagrant rsync\033[0m\033[1;33m to execute a one time sync of the \033[1;2mdockerized-norlab-project\033[0m\033[1;33m directory with all guess VM or use the command \033[1;2mvagrant rsync-auto\033[0m\033[1;33m to start file watching and sync automaticaly on changes. Alternatively, enable \033[1;2mvagrant-gatling-rsync\033[0m\033[1;33m in the Vagrantfile.\033[0m"
  end

end


