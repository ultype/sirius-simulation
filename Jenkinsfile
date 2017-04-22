
def builds = [:]

builds['single-node'] = {
    node{
        def workspace = pwd()

        stage('Single-Node Checkout Source Code') {
            checkout scm
        }

        stage ('Test Single Node') {
            sh '''
                cd exe/single-node
                ./test.sh
                python ../../tools/ci_test.py result.csv 1e-5 > test_result
            '''
             archiveArtifacts artifacts: 'exe/single-node/result.csv, exe/single-node/test_result',
                     fingerprint: true
        }
    }
}

builds['SIL'] = {
    node{
        def workspace = pwd()

        stage('SIL Checkout Source Code') {
            checkout scm
        }

        stage ('Test SIL') {
            sh '''
                cd exe/SIL
                ./test.sh
                python ../../tools/ci_test.py result.csv 1e-5 > test_result
            '''
             archiveArtifacts artifacts: 'exe/SIL/result.csv, exe/SIL/test_result',
                     fingerprint: true
        }
    }
}

builds['PIL'] = {
    node{
        def workspace = pwd()

        stage('PIL Checkout Source Code') {
            checkout scm
        }

        stage ('Test PIL') {
            sh '''
                cd exe/PIL/slave
                trick-CP
                cd ../master
                ./test.sh
                python ../../../tools/ci_test.py result.csv 1e-5 > test_result
            '''
             archiveArtifacts artifacts: 'exe/PIL/master/result.csv, exe/PIL/master/test_result',
                     fingerprint: true
        }
    }
}

builds['Check Style'] = {
    node{
        def workspace = pwd()

        stage('Checkout Source Code') {
            checkout scm
        }

        stage ('Check Style') {
            try {
                sh '''
                    find ./ -regex ".*\\.\\(c\\|h\\|cc\\|hh\\|cpp\\)" | xargs python tools/cpplint.py --counting=detailed --extensions=c,h,cc,hh,cpp --verbose=0 > style_report 2>&1
                '''
            }
            catch (error) {
                sh '''
                    cat style_report
                '''
                throw error
            }
        }
    }
}

parallel builds
