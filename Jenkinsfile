#!/usr/bin/env groovy

def builds = [:]

builds['single-node'] = {
    node {
        def workspace = pwd()

        stage('Single-Node Checkout Source Code') {
            try {
                checkout scm
            }
            catch (error) {
                notifySCMFail()
                throw error
            }
        }

        stage ('Test Single Node') {
            try {
                timeout(10) {  // execute at most 10 minutes
                    sh '''
                        cd exe/single-node
                        ./test.sh
                        python ../../tools/ci_test.py result.csv 1e-5 > test_result
                    '''
                    archiveArtifacts artifacts: 'exe/single-node/result.csv, exe/single-node/test_result',
                                     fingerprint: true
                }
            }
            catch (error) {
                notifyResult('FAILURE')
                throw error
            }
        }
    }
}

builds['SIL'] = {
    node {
        def workspace = pwd()

        stage('SIL Checkout Source Code') {
            try {
                checkout scm
            }
            catch (error) {
                notifySCMFail()
                throw error
            }
        }

        stage ('Test SIL') {
            try {
                timeout(10) {  // execute at most 10 minutes
                    sh '''
                        cd exe/SIL
                        ./test.sh
                        python ../../tools/ci_test.py result.csv 1e-5 > test_result
                    '''
                    archiveArtifacts artifacts: 'exe/SIL/result.csv, exe/SIL/test_result',
                                     fingerprint: true
                }
            }
            catch (error) {
                notifyResult('FAILURE')
                throw error
            }
        }
    }
}

builds['PIL'] = {
    node {
        def workspace = pwd()

        stage('PIL Checkout Source Code') {
            try {
                checkout scm
            }
            catch (error) {
                notifySCMFail()
                throw error
            }
        }

        stage ('Test PIL') {
            try {
                timeout(10) {  // execute at most 10 minutes
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
            catch (error) {
                notifyResult('FAILURE')
                throw error
            }
        }
    }
}

builds['Testing'] = {
    node {
        def workspace = pwd()

        stage('Testing Checkout Source Code') {
            try {
                checkout scm
            }
            catch (error) {
                notifySCMFail()
                throw error
            }
        }

        stage ('Check Style') {
            try {
                sh '''
                    ./tools/lint.sh junit 2> style_report.xml
                '''
            }
            catch (error) {
                notifyResult('FAILURE')
                throw error
            }
            finally {
                junit keepLongStdio: true, testResults: 'style_report.xml'
            }
        }

        stage ('Unit Test') {
            try {
                sh '''
                    git clone https://github.com/google/googletest.git
                    cd unit_test
                    make
                    cd -
                    ./unit_test/timeTest --gtest_output=xml:timeTest.xml
                    ./unit_test/mathTest --gtest_output=xml:mathTest.xml
                '''
            }
            catch (error) {
                notifyResult('FAILURE')
                throw error
            }
            finally {
                step ([
                    $class:         'XUnitPublisher',
                    testTimeMargin: '3000',
                    thresholdMode:  1,
                    thresholds: [
                        [
                            $class:               'FailedThreshold',
                            failureNewThreshold:  '0',
                            failureThreshold:     '0',
                            unstableNewThreshold: '0',
                            unstableThreshold:    '0'
                        ],
                        [
                            $class:               'SkippedThreshold',
                            failureNewThreshold:  '0',
                            failureThreshold:     '0',
                            unstableNewThreshold: '0',
                            unstableThreshold:    '0'
                        ]
                    ],
                    tools: [
                        [
                            $class:                'GoogleTestType',
                            deleteOutputFiles:     true,
                            failIfNotNew:          true,
                            pattern:               'timeTest.xml',
                            skipNoTestFiles:       false,
                            stopProcessingIfError: true
                        ]
                    ]
                ])
            }
        }
    }
}

parallel builds

node { // for success slack notification
    notifyResult('SUCCESS')
}

def notifyResult(String result) {
    def colorCode
    def projectMsg = "Project Name: ${env.JOB_NAME}"
    def resultMsg = "Result: ${result}\nJob-URL: ${env.JOB_URL}\n${env.BUILD_DISPLAY_NAME} Build-URL: ${env.BUILD_URL}"
    def gitMsg = sh returnStdout:true,
                    script: 'git log -1 --pretty=format:"Author: %an%nCommiter: %cn%nCommit Message: %s%nCommit: %h"'
    def msg = "${projectMsg}\n\n${resultMsg}\n\n${gitMsg}"

    if (result == 'SUCCESS') {
        colorCode = '#36A64F'  // color green
    } else if (result == 'FAILURE') {
        colorCode = '#D00000'  // color red
    }

    slackSend(color: colorCode, message: msg)
}

def notifySCMFail() {
    def colorCode = '#D00000'  // color red
    def projectMsg = "Project Name: ${env.JOB_NAME}"
    def resultMsg = "Result: Fail to get source code\nJob-URL: ${env.JOB_URL}"

    def msg = "${projectMsg}\n\n${resultMsg}"

    slackSend(color: colorCode, message: msg)
}
