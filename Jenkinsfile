#!/usr/bin/env groovy

def builds = [:]
def common        // for storing the groovy script

node { // for loading the shared functions
    git 'https://github.com/octoberskyTW/shareGroovy.git'
    common = load 'common.groovy'
}

builds['SIL'] = {
    node {
        def workspace = pwd()

        stage('SIL Checkout Source Code') {
            common.checkoutSource()
        }

        stage ('SIL Testing') {
            try {
                retry(2) {  // retry at most 2 times
                    timeout(10) {  // execute at most 10 minutes
                        sh '''
                            cd exe/SIL/master
                            ./SIL.sh
                        '''
                        archiveArtifacts artifacts: 'exe/SIL/master/result.csv, exe/SIL/master/test_result',
                                         fingerprint: true
                    }
                }
            }
            catch (error) {
                common.notifyResult('FAILURE')
                throw error
            }
        }
    }
}

builds['lint'] = {
    node {
        def workspace = pwd()

        stage('Lint Checkout Source Code') {
            common.checkoutSource()
        }

        stage ('Code Style Checking') {
            try {
                // "lint.sh" generates the xml files "C_style_report" and "Cpp_style_report"
                sh '''
                    ./tools/lint.sh junit
                '''
            }
            catch (error) {
                common.notifyResult('FAILURE')
                throw error
            }
            finally {
                junit keepLongStdio: true, testResults: 'C_style_report'
                junit keepLongStdio: true, testResults: 'Cpp_style_report'
            }
        }
    }
}

parallel builds

node { // for success slack notification
    common.notifyResult('SUCCESS')
}
