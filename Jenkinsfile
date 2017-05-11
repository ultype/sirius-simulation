
def builds = [:]

builds['single-node'] = {
    node {
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
    node {
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
    node {
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
    node {
        def workspace = pwd()

        stage('Lint Checkout Source Code') {
            checkout scm
        }

        stage ('Check Style') {
            try {
                sh '''
                    ./tools/lint.sh junit 2> style_report.xml
                '''
                currentBuild.result = 'SUCCESS'
            }
            catch (error) {
                currentBuild.result = 'FAILURE'
                throw error
            }
            finally {
                junit keepLongStdio: true, testResults: 'style_report.xml'
                notifyBuild(currentBuild.result)
            }
        }
    }
}

parallel builds

def notifyBuild(String buildStatus) {
    def colorCode = '#AAAAAA'
    def projectMsg = "Project Name: ${env.JOB_NAME}"
    def resultMsg = "Result: ${buildStatus}, URL: ${env.BUILD_URL}"
    def msg = "${projectMsg}\n${resultMsg}"

    if (buildStatus == 'SUCCESS') {
        colorCode = '#00FF00'
    } else if (buildStatus == 'FAILURE') {
        colorCode = '#FF0000'
    } else if (buildStatus == 'UNSTABLE') {
        colorCode = '#00FFFF'
    }

    slackSend(color: colorCode, message: msg)
}
