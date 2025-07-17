# -*- mode: ruby -*-
# vi: set ft=ruby :

# =================================================================================================
# dockerized-norlab-project VM for dev and UX validation.
# Support both Ubuntu and MacOsX on arm64 hardware
#
# Usage:
#   $ vagrant up
#
#   To override the vagrant box from command line
#   $ VAGRANT_BOX="bento/ubuntu-24.04" vagrant up
#
# =================================================================================================
ENV["LC_ALL"] = "en_US.UTF-8"

# ====Set which virtual box========================================================================
# ....Ubuntu box...................................................................................
BOX = ENV['VAGRANT_BOX'] || "bento/ubuntu-24.04"
# BOX = ENV['VAGRANT_BOX'] || "bento/ubuntu-22.04" # https://portal.cloud.hashicorp.com/vagrant/discover/bento/ubuntu-22.04

# ....MacOsX box...................................................................................
# BOX = ENV['VAGRANT_BOX'] || "stromweld/macos-15" # https://portal.cloud.hashicorp.com/vagrant/discover/stromweld/macos-15
# BOX = ENV['VAGRANT_BOX'] || "scwx/sequoia-arm" # https://portal.cloud.hashicorp.com/vagrant/discover/scwx/ventura-arm
# BOX = ENV['VAGRANT_BOX'] || "scwx/sequoia" # amd64 version

# ====Begin========================================================================================
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

  # Quick-hack: set 'rsync__chown' to false to delay ownership change and prevent rsync error on MacOsX box
  config.vm.synced_folder ".", "/opt/dockerized-norlab-project",
    type: "rsync",
    rsync__chown: false,
    rsync__exclude: [".idea", ".DS_Store"],
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
    # Enable nested virtualization
    prl.customize ["set", :id, "--nested-virt", "on"]

    # Customize the amount of memory on the VM:
    prl.memory = 16048
    prl.cpus = 4

    # Setup Parallels Tools Auto-Update
    if BOX =~ /ubuntu/
        prl.update_guest_tools = true
    else
        prl.update_guest_tools = false
    end

    # VM portability
    prl.linked_clone = true # (CRITICAL) ToDo: Switch to false for release (!)

    prl.customize "post-import", ["set", :id,
      "--description", "Dockerized-Norlab-Project Virtual Machine",
       "--startup-view", "headless",
      ]
    # GUI cutomizations: --startup-view [window, fullscreen, headless, same] --3d-accelerate highest --high-resolution on
    # Optimization cutomizations: --faster-vm on --resource-quota unlimited
    # Set disk size for new VMs (Note: not working ⚠️): "--device-set", "hdd0", "--size", "50G"

  end

  #config.vm.provider "virtualbox" do |v|
  #  #v.name = "dockerized-norlab-project-DEV"
  #  v.memory = 16048
  #  v.cpus = 4
  #
  #  # VM portability
  #  v.linked_clone = true # (CRITICAL) ToDo: Switch to false for release (!)
  #
  #  # Set disk size for new VMs
  #  v.vm.disk :disk, size: "50GB", primary: true
  #end


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
  config.vm.define "dockerized-norlab-project-vm", primary: true do |dna|
    dna.vm.box = BOX

    # Add these SSH configurations
    dna.ssh.insert_key = true
    dna.ssh.username = "vagrant"
    dna.ssh.password = "vagrant"

    #dna.vm.hostname = "redleader"
    #dna.vm.network "private_network", ip: "10.211.55.99"
    #config.vm.network "forwarded_port", guest: 80, host: 8080, auto_correct: true
    #config.vm.network "forwarded_port", guest: 22, host: 2222

    dna.vm.provision "Check network", privileged: false, type: "shell", run: "always", inline: <<-SHELL
       if [[ "$(uname)" == "Darwin" ]]; then
           # This version find the defaut interface automaticaly -> it is more robust
           echo -e "\nI am '$(whoami)', my address is $(ipconfig getifaddr $(route -n get default | grep interface | awk '{print $2}'))\n"
       else
           echo -e "\nI am '$(whoami)', my address is $(ifconfig eth0 | grep inet | awk '$1==\"inet\" {print $2}')\n"
       fi
    SHELL

  end

  # ====Provisioning========================
  config.vm.provision "shell", privileged: false, inline: <<-SHELL
    if [[ "$(uname)" == "Linux" ]]; then
        :
    elif [[ "$(uname)" == "Darwin" ]]; then
        echo -e "\nInstalling Homebrew as non-root user...\n"
        echo -e "///////////////////////////////////////"
        whoami && uname -a && printenv
        echo -e "///////////////////////////////////////"

        NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" || exit 1
        echo -e "\nDone installing Homebrew\n"

        NON_ROOT_HOME=/Users/vagrant
        echo -e "\nSetup Homebrew for user ${NON_ROOT_HOME}\n"

        if [[ ! -f "${NON_ROOT_HOME}/.zshrc" ]]; then
            sudo touch "${NON_ROOT_HOME}/.zshrc"
        fi
        if [[ ! -f "${NON_ROOT_HOME}/.bashrc" ]]; then
            sudo touch "${NON_ROOT_HOME}/.bashrc"
        fi
        if [[ ! -f "${NON_ROOT_HOME}/.zprofile" ]]; then
            sudo touch "${NON_ROOT_HOME}/.zprofile"
        fi

        echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' | sudo tee -a ${NON_ROOT_HOME}/.bashrc > /dev/null
        echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' | sudo tee -a ${NON_ROOT_HOME}/.zprofile > /dev/null
        echo "source ${NON_ROOT_HOME}/.bashrc" | sudo tee -a ${NON_ROOT_HOME}/.zshrc > /dev/null

        # Load homebrew path without sourcing
        eval "$(/opt/homebrew/bin/brew shellenv)" || { echo "Unable to load Homebreww path!"; exit 1 ; }

        echo -e "\n... Homebrew config ..."
        brew config || exit 1
        echo -e ".......................\n"

        brew update || exit 1
        brew upgrade || exit 1
        brew install git || exit 1
        brew install tree || exit 1

        echo
    fi
  SHELL


  $INLINE_SCRIPT = <<-'SCRIPT'
  echo -e "\nInstalling components in priviledge mode...\n"
  echo -e "///////////////////////////////////////"
  whoami && uname -a && printenv
  echo -e "///////////////////////////////////////"

  if [[ "$(uname)" == "Darwin" ]]; then
      NON_ROOT_HOME=/Users/vagrant

      if [[ -f "${NON_ROOT_HOME:?err}/.zshrc" ]]; then
          source "${NON_ROOT_HOME:?err}/.zshrc" || exit 1
      else
          echo -e "Unable to find ${NON_ROOT_HOME}/.zshrc indicating that the previous provisioning stage might have fail!" 1>&2
          exit 1
      fi
  elif [[ "$(uname)" == "Linux" ]]; then
      NON_ROOT_HOME=/home/vagrant

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
         net-tools \
         rsync \
         dnsutils
  fi

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
  sudo chown -R $(id -u vagrant):$(id -g vagrant) /opt/dockerized-norlab-project-mock-EMPTY

  # Ressetting
  echo -e "\nRessetting .bashrc'\n"
  rm -f /usr/local/bin/dna
  sed -i.bak '/# >>>> dockerized-norlab-project (start)/,/# <<<< dockerized-norlab-project (end)/d' ${NON_ROOT_HOME}/.bashrc
  sed -i.bak '/# >>>> dockerized-norlab-project CUDA (start)/,/# <<<< dockerized-norlab-project CUDA (end)/d' ${NON_ROOT_HOME}/.bashrc
  sed -i.bak '/# >>>> dockerized-norlab-project DEV alias (start)/,/# <<<< dockerized-norlab-project DEV alias (end)/d' ${NON_ROOT_HOME}/.bashrc

  # Set VM dev aliases
  echo -e "\nSet DEV aliases\n"
  {
      echo "" ;
      echo "# >>>> dockerized-norlab-project DEV alias (start)" ;
      echo "alias dna-dna-cd='cd /opt/dockerized-norlab-project'" ;
      #echo "alias dna-mock-cd='cd /opt/dockerized-norlab-project/utilities/tmp/dockerized-norlab-project-mock'" ;
      echo "alias dna-mock-empty-cd='cd /opt/dockerized-norlab-project-mock-EMPTY'" ;
      echo "alias dna-test-symlink='tree -L 1 -a /usr/local/bin/'" ;
      echo "alias dna-test-bashrc='tail -n 20 ~/.bashrc'" ;
      echo "# <<<< dockerized-norlab-project DEV alias (end)" ;
      echo "" ;
  } | sudo tee -a ${NON_ROOT_HOME}/.bashrc > /dev/null

  echo -e "\nSet DEV default landing path to DNA mock super project\n"
  echo "cd /opt/dockerized-norlab-project" >> ${NON_ROOT_HOME}/.bashrc

  if [[ "$(uname)" == "Linux" ]]; then
      # (Priority) ToDo: on task end >> UN-mute those line ↓
      #echo -e "\nInstall The Ubuntu Desktop Gui. Require VM rebooting\n"
      #apt-get install --assume-yes --no-install-recommends ubuntu-desktop || exit 1

      echo "Add second user: vagrant2"
      sudo useradd -m -s /bin/bash vagrant2 2>/dev/null
      echo "vagrant2:vagrant2" | sudo chpasswd

      # Note: ↓↓ Unmute next line to set dockerized-norlab-project ownership to root ↓↓
      #sudo chown -R $(id -u root):$(id -g root) /opt/dockerized-norlab-project

      echo -e "\nProvisioning done.\n"
      source "${NON_ROOT_HOME:?err}/.bashrc"
  elif [[ "$(uname)" == "Darwin" ]]; then
      #sudo bash "/opt/dockerized-norlab-project/tests/vagrant_helper/docker_mock.bash" "${NON_ROOT_HOME}/.bashrc" || exit 1
      echo -e "\nMock docker command as a workaround for MacOsX nested virtualization limitations.\n"
      sudo mkdir -p /usr/local/bin/
      sudo ln -sf /opt/dockerized-norlab-project/tests/vagrant_helper/docker /usr/local/bin/docker
      sudo chmod +x /opt/dockerized-norlab-project/tests/vagrant_helper/docker

      echo -e "\nProvisioning done.\n"
      source "${NON_ROOT_HOME:?err}/.zshrc"
  fi
  SCRIPT

  config.vm.provision :shell do |shell|
    shell.inline = $INLINE_SCRIPT
    shell.privileged = true
    if BOX =~ /ubuntu/
        #shell.reboot = true
        shell.reboot = false
    else
        shell.reboot = false
    end
  end

  # Execute rsync from the host on 'vagrant up' trigger
  config.trigger.after :up do |trigger|
     trigger.only_on = "dockerized-norlab-project-vm"
     trigger.name = "rsync cmd"
     trigger.info = "Executing 'vagrant rsync'"
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

  # Quick-hack for portability on MacOsX box, change ownership on rsync explicitly instead of using 'synced_folder rsync__chown=true'
  config.trigger.after [:up, :reload, :snapshot_restore] do |trigger|
     trigger.only_on = "dockerized-norlab-project-vm"
     trigger.name = "ownership change"
     trigger.info = "change ownership on rsync explicitly instead of using 'synced_folder rsync__chown=true'"
     trigger.run_remote = {privileged: true, inline: "chown -R \$(id -u vagrant):\$(id -g vagrant) /opt/dockerized-norlab-project ; chmod +x /opt/dockerized-norlab-project/tests/vagrant_helper/docker ; echo 'Ownership changed to vargant:vagrant'"}
     trigger.on_error = :continue
  end


  config.trigger.after :up do |trigger|
    trigger.only_on = "dockerized-norlab-project-vm"
    trigger.name = "Dir sync info"
    trigger.info = "\033[1;33m Remember to use the command \033[1;2mvagrant rsync\033[0m\033[1;33m to execute a one time sync of the \033[1;2mdockerized-norlab-project\033[0m\033[1;33m directory with all guess VM or use the command \033[1;2mvagrant rsync-auto\033[0m\033[1;33m to start file watching and sync automaticaly on changes. Alternatively, enable \033[1;2mvagrant-gatling-rsync\033[0m\033[1;33m in the Vagrantfile.\033[0m"
  end

end


