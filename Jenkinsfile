#!/usr/bin/env groovy

def builds = [:]
def common        // for storing the groovy script

node { // for loading the shared functions
    git 'git@tainan.tispace.com:tools/shareGroovy.git'
    common = load 'common.groovy'
}

builds['PIL'] = {
    node {
        def workspace = pwd()

        stage('PIL Checkout Source Code') {
            common.checkoutSource()
        }

        stage ('Test PIL') {
            try {
                retry(2) {  // retry at most 2 times
                    timeout(10) {  // execute at most 10 minutes
                        sh '''
                            cd exe/PIL/master
                            ./test.sh
                        '''
                        archiveArtifacts artifacts: 'exe/PIL/master/result.csv, exe/PIL/master/test_result',
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

builds['Testing'] = {
    node {
        def workspace = pwd()

        stage('Testing Checkout Source Code') {
            common.checkoutSource()
        }

        stage ('Check Style') {
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
