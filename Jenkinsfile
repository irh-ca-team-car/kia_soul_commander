#!groovy
node('xenial') {
  try {
	stage('Checkout') {
	  clean_checkout()
	}
    stage('Build') {
        sh 'git clone https://github.com/boiscljo/oscc.git --branch master && mkdir build && cd build && cmake .. -DVEHICLE=kia_soul_ev && make'
        echo 'Build Complete!'
    }
  }
  finally {
    deleteDir()
  }
}