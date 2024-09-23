CREATE TABLE BATCH_JOB_INSTANCE (
                                    JOB_INSTANCE_ID BIGINT NOT NULL PRIMARY KEY,
                                    VERSION BIGINT,
                                    JOB_NAME VARCHAR(100) NOT NULL,
                                    JOB_KEY VARCHAR(32) NOT NULL,
                                    CONSTRAINT JOB_INST_UN UNIQUE (JOB_NAME, JOB_KEY)
);

CREATE TABLE BATCH_JOB_EXECUTION (
                                     JOB_EXECUTION_ID BIGINT NOT NULL PRIMARY KEY,
                                     VERSION BIGINT,
                                     JOB_INSTANCE_ID BIGINT NOT NULL,
                                     CREATE_TIME DATETIME NOT NULL,
                                     START_TIME DATETIME DEFAULT NULL,
                                     END_TIME DATETIME DEFAULT NULL,
                                     STATUS VARCHAR(10),
                                     EXIT_CODE VARCHAR(2500),
                                     EXIT_MESSAGE VARCHAR(2500),
                                     LAST_UPDATED DATETIME,
                                     JOB_CONFIGURATION_LOCATION VARCHAR(2500) NULL,
                                     CONSTRAINT JOB_INST_EXEC_FK FOREIGN KEY (JOB_INSTANCE_ID)
                                         REFERENCES BATCH_JOB_INSTANCE(JOB_INSTANCE_ID)
);
