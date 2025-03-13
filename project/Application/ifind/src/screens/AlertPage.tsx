import AsyncStorage from "@react-native-async-storage/async-storage";
import firestore, { FirebaseFirestoreTypes } from '@react-native-firebase/firestore';
import React, { useEffect, useState } from "react";
import { Image, ScrollView, StyleSheet, Text, View } from "react-native";
import { SafeAreaProvider } from "react-native-safe-area-context";
import mapdefault from "../styles/mapdefault";
import theme from "../styles/theme";

interface Alert {
    type: number;
    time: FirebaseFirestoreTypes.Timestamp | string;
    stop?: string;
}

export const AlertPage = () => {
    const [alertInfo, setAlertInfo] = useState<Alert[]>([]);

    useEffect(() => {
        const fetchUserInfo = async () => {
            const uid = await AsyncStorage.getItem('userUID');
            if (!uid) return;
    
            const userDocRef = firestore().collection('users').doc(uid);
    
            const unsubscribe = userDocRef.onSnapshot(docSnapshot => {
                if (docSnapshot.exists) {
                    const data = docSnapshot.data();
                    const alertsUpdated: Alert[] = data?.alert.map((alert: any) => {
                        const date = alert.time ? new Date(alert.time.seconds * 1000) : null;
                
                        const formatter = new Intl.DateTimeFormat('ko-KR', {
                        timeZone: 'Asia/Seoul',
                        year: 'numeric', month: '2-digit', day: '2-digit',
                        hour: '2-digit', minute: '2-digit', second: '2-digit',
                        hour12: false
                        });
                
                        return {
                        ...alert,
                        time: date ? formatter.format(date) : 'N/A',
                        stop: alert.stop || data?.stop || 'N/A'
                        };
                    }) || [];
                    setAlertInfo(alertsUpdated);
                    }
                });
    
            return () => unsubscribe();
        };
    
        fetchUserInfo();
    }, []);

    // async function requestUserPermission() {
    //     const authStatus = await messaging().requestPermission();
    //     const enabled =
    //         authStatus === messaging.AuthorizationStatus.AUTHORIZED ||
    //         authStatus === messaging.AuthorizationStatus.PROVISIONAL;
    
    //     if (enabled) {
    //         console.log('Authorization status:', authStatus);
    //         getTokenAndSave();
    //     }
    // }
    
    // async function getTokenAndSave() {
    //     const token = await messaging().getToken();
    //     const uid = await AsyncStorage.getItem('userUID');
    //     if (uid) {
    //         firestore()
    //             .collection('users')
    //             .doc(uid)
    //             .update({ fcmToken: token })
    //             .then(() => console.log('Token saved to Firestore'))
    //             .catch((error) => console.error('Error saving token:', error));
    //     }
    // }
    
    // // 앱이 시작할 때 권한 요청
    // useEffect(() => {
    //     requestUserPermission();
    // }, []);



    return (
        <SafeAreaProvider>
            <View style={styles.container}>
                <Text style={styles.title}>알림 내역</Text>
                <View style={styles.separator} />
                <ScrollView style={styles.alertList}>
                    {alertInfo.map((alert, index) => (
                        <View key={index} style={styles.alertBox}>
                            <Image
                                source={
                                    alert.type === 1
                                        ? require('../asset/home_icon.png')
                                        : require('../asset/school_icon.png')
                                }
                                style={styles.alertIcon}
                            />
                            <View style={styles.textContainer}>
                                <Text style={styles.alertTitle}>{alert.type === 1 ? alert.stop : mapdefault.destination}에 도착했습니다</Text>
                                <Text style={styles.alertTime}>{` ${alert.time}`}</Text>
                            </View>
                        </View>
                    ))}
                </ScrollView>
            </View>
        </SafeAreaProvider>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        alignItems: 'center',
        backgroundColor: '#fff',
    },
    title: {
        paddingTop: 10,
        margin: 10,
        fontSize: 35,
        // fontWeight: "bold",
        fontFamily: theme.mainfont,
        color: theme.mainDarkGrey,
    },
    separator: {
        width: '90%',
        height: 1,
        backgroundColor: theme.navicolor,
        marginBottom: 10,
    },
    alertList: {
        width: '90%',
    },
    textContainer: {
        flex: 1,
        justifyContent: 'center',
        color: theme.mainDarkGrey,
    },
    alertBox: {
        flexDirection: 'row',
        padding: 10,
        marginBottom: 10,
        borderBottomWidth: 0.5,
        borderColor: 'gray',
        alignItems: 'center',
    },
    alertIcon: {
        width: 45,
        height: 45,
        marginRight: 10,
    },
    alertTitle: {
        fontSize: 23,
        // fontWeight: 'bold',
        color: theme.mainDarkGrey,
        fontFamily: theme.mainfont,
    },
    alertTime: {
        fontSize: 17,
        color: theme.mainDarkGrey,
    },
});