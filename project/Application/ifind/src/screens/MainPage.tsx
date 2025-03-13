import AsyncStorage from "@react-native-async-storage/async-storage";
import firestore from '@react-native-firebase/firestore';
import { NativeStackNavigationProp } from "@react-navigation/native-stack";
import React, { useEffect, useRef, useState } from "react";
import { Image, PermissionsAndroid, Platform, StyleSheet, Text, TouchableOpacity, View } from "react-native";
import MapView, { Marker, Polyline } from "react-native-maps";
import { SafeAreaProvider } from "react-native-safe-area-context";
import mapdefault from "../styles/mapdefault";
import theme from "../styles/theme";

interface MainPageProps {
    navigation: NativeStackNavigationProp<any, 'MainPage'>;
}
interface Location {
    latitude: number;
    longitude: number;
}

interface Stop extends Location {
    id: number;
}

export const MainPage = ({ navigation }: MainPageProps) => {
    const [buslocation, setBuslocation] = useState<Location>({ latitude: mapdefault.latitude, longitude: mapdefault.longitude });
    const [path, setPath] = useState<Location[]>([]);
    const [stops, setStops] = useState<Stop[]>([]);
    const mapViewRef = useRef<MapView>(null);
    const [schoollocation, setSchoollocation] = useState<Location>({ latitude: mapdefault.latitude, longitude: mapdefault.longitude });

    //거리 계산 함수
    function getDistance(lat1: number, lon1: number, lat2: number, lon2: number): number {
        var R = 6371; // 지구의 반경(km)
        var dLat = (lat2 - lat1) * Math.PI / 180;
        var dLon = (lon2 - lon1) * Math.PI / 180;
        var a =
            Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
        var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        var d = R * c; // 거리(km)
        return d;
    }

    useEffect(() => {
        const fetchBusLocation = async () => {
            const uid = await AsyncStorage.getItem('userUID');
            if (uid) {
                const unsubscribe = firestore().collection('morai').doc('location').onSnapshot(doc => {
                    const data = doc.data();
                    if (data) {
                        const { latitude, longitude } = data;
                        if (typeof latitude === 'number' && typeof longitude === 'number') {
                            setBuslocation({ latitude, longitude });
                        }
                    } else {
                        console.log("버스 위치를 찾을 수 없습니다.");
                    }
                }, error => {
                    console.error("버스 위치 정보 조회 중 오류 발생:", error);
                });
                return () => unsubscribe();
            }
        };

        const fetchSchool = () => {
            const unsubscribe = firestore().collection('morai').doc('start').onSnapshot(doc => {
                const data = doc.data();
                if (data) {
                    const { latitude, longitude } = data;
                    if (typeof latitude === 'number' && typeof longitude === 'number') {
                        setSchoollocation({ latitude, longitude });
                    }
                } else {
                    console.log("학원 위치를 찾을 수 없습니다.");
                }
            }, error => {
                console.error("학원 위치 정보 조회 중 오류 발생:", error);
            });
            return () => unsubscribe();
        };
    
    
        const fetchPath = () => {
            const unsubscribe = firestore().collection('morai').doc('path').onSnapshot(doc => {
                const data = doc.data();
                if (data && Array.isArray(data.path)) {
                    setPath(data.path as Location[]);
                }
            }, error => {
                console.error("경로 정보 조회 중 오류 발생:", error);
            });
            return () => unsubscribe();
        };
    
        const fetchStops = () => {
            firestore().collection('morai').doc('order').onSnapshot(doc => {
                const data = doc.data();
                if (data && Array.isArray(data.stop)) {
                    const formattedStops = data.stop.map((stop, index) => ({
                        ...stop,
                        id: index + 1
                    }));
                    setStops(formattedStops);
                }
            }, error => {
                console.error("정류장 정보 조회 중 오류 발생:", error);
            });
        };
    
        fetchBusLocation();
        fetchPath();
        fetchSchool();
        fetchStops();
    }, []);

    useEffect(() => {
        const requestLocationPermission = async () => {
            if (Platform.OS === 'android') {
                const granted = await PermissionsAndroid.request(
                    PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
                    {
                        title: "위치 정보 접근 권한",
                        message: "이 앱은 위치 정보 접근 권한이 필요합니다.",
                        buttonNeutral: "나중에 묻기",
                        buttonNegative: "거부",
                        buttonPositive: "허용"
                    }
                );
                return granted === PermissionsAndroid.RESULTS.GRANTED;
            }
            return true;
        };

        // const initializeLocation = async () => {
        //     const hasPermission = await requestLocationPermission();
        //     if (hasPermission) {
        //         Geolocation.watchPosition(
        //             position => {
        //                 const newBusLocation = {
        //                     latitude: position.coords.latitude,
        //                     longitude: position.coords.longitude
        //                 };
        //                 setBuslocation(newBusLocation);
        //                 updatePathInFirebase(newBusLocation); // Firebase에서 경로 업데이트
        //             },
        //             error => {
        //                 console.error(error);
        //             },
        //             { enableHighAccuracy: true, distanceFilter: 1 }
        //         );
        //     } else {
        //         console.log("위치 정보 접근 권한이 거부되었습니다.");
        //     }
        // };

        // initializeLocation();
    }, [path]);

    // const updatePathInFirebase = async (newBusLocation : Location) => {
    //     const pathDoc = await firestore().collection('morai').doc('path').get();
    //     let currentPath = pathDoc.data()?.path;

    //     if (currentPath) {
    //         const updatedPath = currentPath.filter((point: Location) => {
    //             const distance = getDistance(
    //                 newBusLocation.latitude,
    //                 newBusLocation.longitude,
    //                 point.latitude,
    //                 point.longitude
    //             );
    //             return distance > 0.05;
    //         });

    //         await firestore().collection('morai').doc('path').update({
    //             path: updatedPath
    //         });

    //         setPath(updatedPath); // 상태 업데이트
    //     }
    // };

    const moveToBusLocation = () => {
        if (mapViewRef.current) {
            mapViewRef.current.animateToRegion({
                latitude: buslocation.latitude,
                longitude: buslocation.longitude,
                latitudeDelta: mapdefault.latitudeDelta,
                longitudeDelta: mapdefault.longitudeDelta,
            }, 1000);
        }
    };

    return (
        <SafeAreaProvider>
            <View style={styles.container}>
                <Text style={styles.title}>학원버스 위치보기</Text>
                <View style={styles.separator} />
                <MapView
                    ref={mapViewRef}
                    style={styles.map}
                    initialRegion={{
                        latitude: buslocation.latitude,
                        longitude: buslocation.longitude,
                        latitudeDelta: mapdefault.latitudeDelta,
                        longitudeDelta: mapdefault.longitudeDelta,
                    }}
                    // region={{
                    //     latitude: buslocation.latitude,
                    //     longitude: buslocation.longitude,
                    //     latitudeDelta: mapdefault.latitudeDelta,
                    //     longitudeDelta: mapdefault.longitudeDelta,
                    // }}
                    >
                        <Marker
                            coordinate={{
                                latitude: buslocation.latitude,
                                longitude: buslocation.longitude
                            }}
                            // title="약 N분 후 도착"
                            anchor={{ x: 0.5, y: 0.5 }}
                            >
                                <Image
                                    source={require("../asset/bus_icon3.png")}
                                    style={styles.busicon}
                                    resizeMode="contain"
                                />
                        </Marker>
                        <Marker
                            coordinate={{
                                latitude: schoollocation.latitude,
                                longitude: schoollocation.longitude
                            }}
                            title="약 N분 후 도착"
                            anchor={{ x: 0.5, y: 0.5 }}
                            >
                                <Image
                                    source={require("../asset/start_icon.png")}
                                    style={styles.busicon}
                                    resizeMode="contain"
                                />
                        </Marker>
                        {path && (
                        <Polyline
                            coordinates={path.map(p => ({
                                latitude: p.latitude,
                                longitude: p.longitude,
                            }))}
                            strokeColor="#007bff"
                            strokeWidth={8}
                        />
                    )}
                    {stops.map((stop, index) => (
                        <Marker
                        key={index}
                        coordinate={{ latitude: stop.latitude, longitude: stop.longitude }}
                        // title={`Stop ${stop.id}`}
                    >
                        <View style={styles.stopMarker}>
                            <Text style={styles.stopText}>{stop.id}</Text>
                        </View>
                    </Marker>
                    ))}
                    </MapView>
                    <TouchableOpacity
                    style={styles.busLocation}
                    onPress={moveToBusLocation}
                >
                    <Text style={styles.busLocationText}>버스 위치로 이동</Text>
                </TouchableOpacity>
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
    title:{
        paddingTop: 10,
        margin: 10,
        fontSize: 35,
        // fontWeight: "bold",
        color: theme.mainDarkGrey,
        fontFamily: theme.mainfont,
    },
    map: {
        margin: 10,
        width: '90%',
        height: '75%',
        backgroundColor: 'grey',
    },
    separator: {
        width: '90%',
        height: 1,
        backgroundColor: theme.navicolor,
        marginBottom: 5,
    },
    busicon: {
        width: 50,
        height: 50,
    },
    stopMarker: {
        backgroundColor: 'red',
        borderRadius: 15,
        padding: 5,
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        width: 30,
        height: 30,
    },
    stopText: {
        fontFamily: theme.mainfont,
        color: '#ffffff',
        fontSize: 20,
        // fontWeight: 'bold',
    },
    busLocation:{
        marginTop: 30,
        width: '50%',
        alignItems: 'center',
        backgroundColor: theme.mainOrange,
        paddingHorizontal: 20,
        paddingVertical: 10,
        borderRadius: 40,
        shadowColor: "#000",
            shadowOffset: {
                width: 0,
                height: 2,
            },
            shadowOpacity: 0.25,
            shadowRadius: 4,
            elevation: 5,
            bottom: '4%',

    },
    busLocationText:{
        padding: 5,
        fontFamily: theme.mainfont,
        color: theme.mainDarkGrey,
        fontSize: 23,
    },
});
